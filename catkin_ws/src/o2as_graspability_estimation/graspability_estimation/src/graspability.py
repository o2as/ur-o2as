import numpy as np
from numpy import sin, cos
from scipy.ndimage import imread
from scipy.ndimage.morphology import binary_erosion
from scipy.ndimage.filters import maximum_filter
#from scipy.ndimage.measurements import label
from scipy.signal import fftconvolve
from scipy.misc import imrotate
from skimage.feature import canny
from skimage.measure import regionprops, label
from skimage.transform import hough_circle, hough_circle_peaks
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from time import time

def create_suction_model(radius):
  """Create a suction model"""
  hm = np.zeros((2 * radius + 1, 2 * radius + 1))

  hm1 = np.tile(np.arange(-radius, radius + 1), (2 * radius + 1, 1))
  hm2 = hm1.T
  d = np.sqrt(hm1**2 + hm2**2)

  return np.where(d < radius, 1, 0).astype(np.float64)

def two_finger_gripper_model(ow, ft, fw):
  w = max(ow + 2 * ft, fw) + 2
  hm1 = np.zeros((w, w))
  hm2 = np.zeros((w, w))

  c = int(w / 2.0)
  how = max(int(ow / 2.0), 1)
  hft = max(int(ft / 2.0), 1)
  hfw = max(int(fw / 2.0), 1)

  hm1[(c - hft):(c + hft), (c - how):(c + how)] = 1
  hm2[(c - hfw):(c + hfw), (c - how - hft):(c - how)] = 1
  hm2[(c - hfw):(c + hfw), (c + how):(c + how + hft)] = 1

  return hm1, hm2

def imregionalmax(img, footprint_size):
  """MATLAB equivalent to imregionalmax
  see https://stackoverflow.com/questions/27598103
  """
  lm = maximum_filter(img, size=footprint_size)
  return np.where(img == lm, 1, 0) # // convert local max values to binary mask

def pcrot(img, rot):
  """Depth image rotation
  """
  # Rotation matrix
  tx, ty, tz = np.deg2rad(rot) * -1

  rx = np.array([[       1,        0,        0, 0],
                 [       0,  cos(tx), -sin(tx), 0],
                 [       0,  sin(tx),  cos(tx), 0],
                 [       0,        0,        0, 1]])
  ry = np.array([[ cos(ty),        0,  sin(ty), 0],
                 [       0,        1,        0, 0],
                 [-sin(ty),        0,  cos(ty), 0],
                 [       0,        0,        0, 1]])
  rz = np.array([[ cos(tz), -sin(tz),        0, 0],
                 [ sin(tz),  cos(tz),        0, 0],
                 [       0,        0,        1, 0],
                 [       0,        0,        1, 1]])
  rt = rz.dot(ry).dot(rx)

  # from depth image to pointcloud
  # NOTE: It looks that this code assume that each pixel in the given depth
  # image is 1 x 1 mm in size.
  h, w = img.shape
  matx = np.tile(np.arange(1, w + 1), (h, 1)).reshape(-1)
  maty = np.tile(np.arange(1, h + 1), (w, 1)).T.reshape(-1)
  xyzo = np.vstack([matx, maty, img.reshape(-1) * 1000, np.ones(h * w)])

  # apply rotation; multiply the rotation matrix from left
  xyz = rt.dot(xyzo)[:3, :]
  assert(xyz.shape == (3, h * w))

  # extract depth information
  return xyz[2, :].reshape(h, w) / 1000.0 # mm -> m

def surfnorm(img):
  """Estimate surface normal from depth image.

  Code is adapted from an OpenCV answer:
  http://answers.opencv.org/question/82453
  (Answer1, Solution1)

  :param img:
  :type img: numpy.ndarray, shape=(height, width)
  """
  assert(len(img.shape) is 2)
  h, w = img.shape

  dzdx = (img[2:, 1:-1] - img[0:-2, 1:-1]) / 2.0
  dzdy = (img[1:-1, 2:] - img[1:-1, 0:-2]) / 2.0
  nmat = np.stack([-dzdx, -dzdy, np.ones((h - 2, w - 2))], axis=2)
  norm = np.linalg.norm(nmat, axis=2)
  nmat = nmat / norm[:, :, np.newaxis]
  pad0 = np.zeros((nmat.shape[0], 1, 3))
  nmat = np.hstack([pad0, nmat, pad0])
  pad1 = np.zeros((1, nmat.shape[1], 3))
  nmat = np.vstack([pad1, nmat, pad1])

  # # debug
  # plt.figure()
  # plt.hist(dzdx.reshape(-1))
  # plt.show()

  return nmat[:, :, 0], nmat[:, :, 1], nmat[:, :, 2]

def find_circles(img, hough_radii, total_num_peaks, vis=False):
  edge = canny((img / np.max(img) * 255).astype(np.uint8), sigma=3)
  if vis:
    plot_img("Canny (edge)", edge.astype(np.int), 4)

  hough_res = hough_circle(edge, hough_radii)
  _, cx, cy, radii = hough_circle_peaks(hough_res, hough_radii,
                                        total_num_peaks=total_num_peaks)
  centers = np.vstack((cx, cy)).T
  assert(len(centers.shape) is 2 and centers.shape[1] == 2)

  return centers, radii

row_min = 0
row_max = 1544
col_min = 0
col_max = 2064

def plot_img(title, img, subplot, cb_off=False, vrange=None):
  plt.subplot(2, 4, subplot)

  if vrange is None:
    plt.imshow(img)
  else:
    plt.imshow(img, vmin=vrange[0], vmax=vrange[1])
  plt.xlim(col_min, col_max)
  plt.ylim(row_max, row_min)
  if not cb_off:
    plt.colorbar()
  plt.title(title)

# 4 parameters each of parts
# radius    : radius of the suction pad(pixel)
# obj_size  : approximate size of a target parts
# open_width: open width of the gripper(pixel)
# d         : depth for insertion of a two-finger gripper (mm)
parts_params = {
   4: ("Target: Geared motor                  ", 10,  6, 100, 3),
   5: ("Target: Pully for round belt          ",  6,  8,  45, 3),
   6: ("Target: Polyurethane for round belt   ",  2,  2,  20, 5),
   7: ("Target: Bearing with housing          ", 12, 12,  50, 1),
   8: ("Target: Drive shaft                   ",  2,  2,  22, 4),
   9: ("Target: End cap for shaft             ",  3,  3,  20, 1),
  10: ("Target: Bearing spacers for inner ring",  3,  3,  20, 1),
  11: ("Target: Pully for round belts clamping",  8,  8,  20, 1),
  12: ("Target: Bearing spacer for inner ring ",  2,  4,  20, 1),
  13: ("Target: Idler for round belt          ",  7,  7,  30, 1),
  14: ("Target: Bearing shaft screw           ",  2,  2,  14, 5),
  15: ("Target: Hex nut                       ",  3,  3,  20, 1),
  16: ("Target: Flat washer                   ",  3,  3,  15, 1),
  17: ("Target: Head cap screw M4             ",  1,  1,  10, 1),
  18: ("Target: Head cap screw M3             ",  1,  1,  10, 1),
}

def _find_centroids(img, imgt, hm, ml, obj_size, footprint_size, imtm, vis):
  """This function is used for suction and two_finger."""
  # find positions which can be contacted by the suction gripper
  t_start = time()

  t_start_tmp = time()
  hm = hm.astype(np.int32)
  imgt = imgt.astype(np.int32)
  tmp = fftconvolve(imgt, hm, "same")
  tmp = np.round(tmp).astype(np.int)
  emap1 = np.where(tmp == hm.sum(), 1, 0)
  print("Elapsed time for fftconvolve: {}".format(time() - t_start_tmp))

  # erode for noise reduction
  t_start_tmp = time()
  emap1 = binary_erosion(emap1, np.ones((3, 3)).astype(np.int))
  print("Elapsed time for erosion: {}".format(time() - t_start_tmp))
  if vis:
    plot_img("imgt", imgt, 7)
    #plot_img("emap1", emap1, 8)

  # estimated graspable objects
  t_start_tmp = time()
  emap2 = fftconvolve(emap1, create_suction_model(obj_size), 'same')
  emap2 = np.round(emap2).astype(np.int)
  if vis:
    plot_img("Conv2 (emap2)", emap2, 4)
  print("Elapsed time for fftconvolve: {}".format(time() - t_start_tmp))

  # graspability
  gb = emap2

  # regionalmax
  t_start_imrmax = time()
  gpeaks = imregionalmax(gb, footprint_size)
  gpeaks = np.where((gpeaks * emap1 * imtm) != 0, 1, 0)
  print("Elapsed time for imregionalmax: {}".format(time() - t_start_imrmax))
  if vis:
    plot_img("Binarized regional max (fp={})".format(footprint_size),
             gpeaks, 5)

  # find centroids from the peaks
  t_start_rprops = time()
  props = regionprops(label(gpeaks))
  n_grasp_points = len(props)
  if n_grasp_points == 0:
    print("ERROR; no graspable points")
    return None
  print("Elapsed time for regionprops: {}".format(t_start_rprops))

  centroids = np.array([p.centroid[:2] for p in props]).astype(np.int32)
  orientations = np.array([p.orientation for p in props])
  c = centroids[:, 1] # column of the image matrix
  r = centroids[:, 0] # row of the image matrix
  print("Found {} centroids".format(len(c)))

  # check positions of graspable points
  t_start_checkpos = time()
  img_width = img.shape[1]
  img_height = img.shape[0]
  for k in range(n_grasp_points):
    c_k, r_k = c[k], r[k]

    # shift each graspable point if required (i.e., gpeaks[r_k, c_k] == 0)
    if gpeaks[r_k, c_k] == 0:
      # extract search area
      sc_k = np.arange(max(0, c_k - ml), min(img_width, c_k + ml + 1))
      sr_k = np.arange(max(0, r_k - ml), min(img_height, r_k + ml + 1))
      lc = len(sc_k)
      lr = len(sr_k)
      sc_k = np.tile(sc_k, (lr, 1)).reshape(-1)
      sr_k = np.tile(sr_k, (lc, 1)).T.reshape(-1)

      # Get region where gpeaks is 1
      ixs = np.where(gpeaks[sr_k, sc_k] == 1)[0]
      if len(ixs) == 0:
        continue

      # Get point nearest to the center
      d = (sc_k[ixs] - c_k) ** 2 + (sr_k[ixs] - r_k) ** 2
      ix_min = np.argmin(d)

      c[k] = sc_k[ixs[ix_min]]
      r[k] = sr_k[ixs[ix_min]]

  print("Elapsed time for check positions: {}".format(time() - t_start_checkpos))
  print("Elapsed time for find_centroids(): {}".format(time() - t_start))

  # emap1 will be used in later
  return c, r, orientations

def _graspability_suction(img, imgt, imrot3, ml, hm, obj_size,
                          nx, ny, nz, footprint_size, imtm, vis,
                          x_min, x_max, y_min, y_max):
  # find centroids from the peaks
  x, y, orientations = _find_centroids(
    img, imgt, hm, ml, obj_size, footprint_size, imtm, vis
  )

  t_start = time()

  n_grasp_points = len(x)

  # depth at each centroid
  dpt = np.zeros(n_grasp_points)
  for k in range(n_grasp_points):
    if x[k] != 0 and y[k] != 0: # TODO: confirm if this check is necessary
      dpt[k] = imrot3[y[k], x[k]]
    else:
      dpt[k] = 0

  # sorting (ascending) and filtering out zero entries
  b = np.argsort(dpt)
  a = dpt[b]
  ix_nz = np.where(a != 0)[0]
  a, b = a[ix_nz], b[ix_nz]

  # results for a suction gripper
  gscore = a
  posx = x[b]
  posy = y[b]
  posz = img[posy, posx]
  rotx = nx[posy, posx]
  roty = ny[posy, posx]
  rotz = nz[posy, posx]
  # To match with the robot coordinate system, multiply each value by -1.
  # 2018/10/11 Yuma Hijioka
  rotipz = orientations[b] * -1
  rotipz = rotipz / np.pi * 180.0

  # Convert score; higher is better
  gscore = 2.0 - gscore / np.min(gscore)

  print("Elapsed time for post processing: {}".format(time() - t_start))

  if vis:
    plot_img("Result ({} points found)".format(len(x)), imrot3, 6,
             cb_off=True)
    plt.scatter(posx, posy, s=10, c=gscore, cmap="Reds")
    plt.colorbar()

  # Create image to output
  # https://stackoverflow.com/questions/35355930
  fig = plt.figure()
  canvas = FigureCanvas(fig)

  plt.imshow(imrot3)
  plt.scatter(posx, posy, s=10, c=gscore, cmap="Reds")
  plt.xlim(x_min, x_max)
  plt.ylim(y_max, y_min)

  plt.colorbar()

  canvas.draw()  # draw the canvas, cache the renderer

  width, height = fig.get_size_inches() * fig.get_dpi()
  width, height = int(width), int(height)
  image = np.fromstring(canvas.tostring_rgb(), dtype='uint8')
  image = image.reshape(height, width, 3)

  return posx, posy, posz, rotx, roty, rotz, rotipz, gscore, image

def _graspability_two_finger(img, imgt, imrot3, img_rot, ml, hm, obj_size, d,
                             nx, ny, nz, footprint_size, imtm, hm1, hm2,
                             numo, vis, x_min, x_max, y_min, y_max):
  # find centroids from the peaks
  x, y, orientations = _find_centroids(
    img, imgt, hm, ml, obj_size, footprint_size, imtm, vis
  )
  n_grasp_points = len(x)

  # depth at each centroid
  dpt = np.zeros(n_grasp_points)
  dpt2 = np.zeros(n_grasp_points)
  for k in range(n_grasp_points):
    if x[k] != 0 and y[k] != 0: # TODO: confirm if this check is necessary
      dpt[k] = imrot3[y[k], x[k]]
      dpt2[k] = img_rot[y[k], x[k]]
    else:
      dpt[k] = 0
      dpt2[k] = 0

  # collision check for the gripper
  hs = hm1.shape[0] / 2.0
  gg = np.zeros(n_grasp_points)
  gang = np.zeros(n_grasp_points).astype(np.int)
  theta = np.arange(0, 180.0, 180.0 / numo)

  # size check
  for i in range(numo):
    hmo2 = imrotate(hm2, theta[i])
    tmp = hmo2.shape[0] / 2.0
    if tmp > hs:
      hs = tmp
  hsb = hs

  for k in range(n_grasp_points):
    x_k, y_k = x[k], y[k]

    if (x_k >= hsb) and (x_k - hsb < img.shape[1]) and \
       (y_k >= hsb) and (y_k - hsb < img.shape[0]):
      for i in range(numo):
        hmo2 = imrotate(hm2, theta[i])
        hs = hmo2.shape[1] / 2
        xk0 = int(x_k - hs)
        xk1 = int(x_k + hs)
        yk0 = int(y_k - hs)
        yk1 = int(y_k + hs)
        tmp = img_rot[yk0:yk1, xk0:xk1]
        cm = np.where(tmp >= dpt2[k] - d / 1000.0, 1, 0)
        tmp2 = cm * hmo2

        if np.sum(tmp2) == 0:
          gg[k] = 1
          gang[k] = i

  ixs = np.where(gg == 1)[0]

  if len(ixs) == 0:
    return None

  gscore = dpt[ixs]
  gang = gang[ixs]
  posx = x[ixs]
  posy = y[ixs]
  posz = img[posy, posx]
  rotx = nx[posy, posx]
  roty = ny[posy, posx]
  rotz = nz[posy, posx]
  # To match with the robot coordinate system, multiply each value by -1.
  # 2018/10/11 Yuma Hijioka
  rotipz = theta[gang] * -1 

  # Convert score; higher is better
  gscore = 2.0 - gscore / np.min(gscore)

  if vis:
    degs = theta[gang]
    rads = np.pi * degs / 180
    plot_img("Result ({} points found)".format(len(posx)), imrot3, 6,
             cb_off=True)
    plt.scatter(posx, posy, s=10, c=gscore, cmap="Reds")
    for x, y, rad in zip(posx, posy, rads):
      ow = 30 # for display
      plt.plot([x - ow / 2 * np.cos(rad), x + ow / 2 * np.cos(rad)],
               [y - ow / 2 * np.sin(rad), y + ow / 2 * np.sin(rad)],
               color="r")
    plt.colorbar()

  # Create image to output
  # https://stackoverflow.com/questions/35355930
  fig = plt.figure()
  canvas = FigureCanvas(fig)

  plt.imshow(imrot3)
  degs = theta[gang]
  rads = np.pi * degs / 180
  plt.scatter(posx, posy, s=10, c=gscore, cmap="Reds")
  for x, y, rad in zip(posx, posy, rads):
    ow = 30  # for display
    plt.plot([x - ow / 2 * np.cos(rad), x + ow / 2 * np.cos(rad)],
             [y - ow / 2 * np.sin(rad), y + ow / 2 * np.sin(rad)],
             color="r")
  plt.xlim(x_min, x_max)
  plt.ylim(y_max, y_min)
  plt.colorbar()

  canvas.draw()  # draw the canvas, cache the renderer

  width, height = fig.get_size_inches() * fig.get_dpi()
  width, height = int(width), int(height)
  image = np.fromstring(canvas.tostring_rgb(), dtype='uint8')
  image = image.reshape(height, width, 3)

  return posx, posy, posz, rotx, roty, rotz, rotipz, gscore, image

def _graspability_inner(img, imgt, img_rot, hm, nx, ny, nz, hough_radii,
                        total_num_peaks, imrot3, vis, x_min, x_max, y_min,
                        y_max):
  # Find holes
  centers, radii = find_circles(imgt, hough_radii, total_num_peaks, vis)
  n_holes = centers.shape[0]
  print("{} holes detected".format(n_holes))

  # Collision check
  hs = hm.shape[0] / 2
  # gg = np.zeros(centers.shape[0]) # this variable is not used
  radm = np.mean(radii)
  dpt = np.zeros(n_holes)
  dpt2 = np.zeros(n_holes)
  mtmp = np.zeros(n_holes)

  for i in range(n_holes):
    cx = int(centers[i, 0])
    cy = int(centers[i, 1])

    if (cy - radm) > 0 and (cy + radm) < img.shape[0] and \
       (cx - radm) > 0 and (cx + radm) < img.shape[1]:
      tmprot = 2 * np.pi * np.arange(100) / 100.0
      tmp = img_rot[(cy + np.cos(tmprot) * radm).astype(np.int),
                    (cx + np.sin(tmprot) * radm).astype(np.int)]
      ixs = np.where(tmp != 0)[0]
      dpt[i] = np.mean(tmp[ixs]) if len(ixs) > 0 else 0
      dpt2[i] = img[int(centers[i, 1]), int(centers[i, 0])]

      # hole depth
      cx0 = int(cx - hs)
      cx1 = int(cx + hs)
      cy0 = int(cy - hs)
      cy1 = int(cy + hs)
      tmp = img_rot[cy0:cy1, cx0:cx1]
      ixs = np.where(tmp.reshape(-1) > 0)[0]
      mtmp[i] = 0 if len(ixs) == 0 else np.mean(tmp.reshape(-1)[ixs])

      if (dpt[i] < mtmp[i]):
        dpt[i] = 0

    else:
      dpt[i] = 0

  ixs = np.argsort(-1 * dpt)
  ixs = ixs[np.where(dpt[ixs] > 0)[0]]
  gscore = dpt[ixs]
  posx = centers[ixs, 0]
  posy = centers[ixs, 1]
  posz = dpt[ixs]
  rotx = nx[posy, posx]
  roty = ny[posy, posx]
  rotz = nz[posy, posx]
  rotipz = np.zeros(len(ixs))

  # Normalize
  m = np.max(gscore)
  gscore = gscore / m if m > 0 else gscore

  if vis:
    print(radii)
    circles = [plt.Circle(c, r, color="r", fill=False)
               for c, r in zip(centers, radii)]
    plot_img("Result ({} points found)".format(len(posx)), imrot3, 6,
             cb_off=True)
    plt.scatter(posx, posy, s=10, c=gscore, cmap="Reds")
    for c in circles:
      plt.gca().add_artist(c)
    plt.colorbar()

  # Create image to output
  # https://stackoverflow.com/questions/35355930
  fig = plt.figure()

  canvas = FigureCanvas(fig)

  plt.imshow(imrot3)
  circles = [plt.Circle(c, r, color="r", fill=False)
             for c, r in zip(centers, radii)]
  plt.scatter(posx, posy, s=10, c=gscore, cmap="Reds")
  for c in circles:
    plt.gca().add_artist(c)
  plt.xlim(x_min, x_max)
  plt.ylim(y_max, y_min)
  plt.colorbar()

  canvas.draw()  # draw the canvas, cache the renderer

  width, height = fig.get_size_inches() * fig.get_dpi()
  width, height = int(width), int(height)
  image = np.fromstring(canvas.tostring_rgb(), dtype='uint8')
  image = image.reshape(height, width, 3)

  return posx, posy, posz, rotx, roty, rotz, rotipz, gscore, image

def graspability(img, parts_id, bin_id, gripper_type, mask_image,
                 vis=False, footprint_size=30, ignore_mask=False):
  """
  :param im:
  :param parts_id: Parts ID which starts from 1 (not 0).
  :param bin_id: Bin ID which starts from 1 (not 0).
  :param gripper_type:
  :return:
  """
  # Roudh validation of input: diff of neighbor pixels should be less than
  # 3 meter
  if np.max(np.abs(img[1:, :] - img[:-1, :])) >= 3.0:
    print("Error: input depth image is not possibly be in the unit of meter.")
    return None

  t_start = time()

  # Phoxi's rotation angle (deg) on each axis(X,Y,Z)
  rot = np.array([-7.5, 6.0, 0])

  # filter size for erode (noise reduction)
  ns = 0 if parts_id in {8, 16, 14, 17, 18} else 2

  # a threshold for background subtraction (mm)
  bl = 3

  # size of a search area for shifting graspable positions
  ml = 50

  # finer thickness for a two finger gripper (pixel)
  ft = 1

  # finger width for a two finger gripper (pixel)
  fw = 5

  # the number of orientation of a two-finger gripper
  numo = 8

  # type of a gripper
  if gripper_type is "suction":
    print("Gripper type: suction")
  elif gripper_type is "two_finger":
    print("Gripper type: two-finger")
  elif gripper_type is "inner":
    print("Gripper type: two-finger (inner)")
  else:
    print("Error: undefined gripper type")
    return None

  # params which should be modified for each parts
  # obj_size  : approximate size of a target parts
  # radius    : radius of the suction pad(pixel)
  # open_width: open width of the gripper(pixel)
  # d         : depth for insertion of a two-finger gripper (mm)
  msg, radius, obj_size, open_width, d = parts_params[parts_id]
  if gripper_type is "inner":
    radius = 8

  # Parameters for circle detection, inputted to hough_circle
  # These parameters affects the algo when gripper_type is "inner"
  hough_radii = np.arange(12, 32, 2)
  total_num_peaks = 50

  # create a suction model
  hm = create_suction_model(radius)

  # create a two-finger gripper model
  hm1, hm2 = two_finger_gripper_model(open_width, ft, fw)

  # mask image
  imr = mask_image
  assert((len(imr.shape) == 2) or (imr.shape[2] == 1))

  # select an area of the target bin
  if bin_id == 0:
    imr = np.where(imr > 0, 1, 0)
  else:
    imr = np.where(imr == bin_id, 1, 0)

  # Region to be returned: x_min, x_max, y_min, y_max
  tmp_nz = np.where(imr.sum(axis=0) != 0)[0]
  if len(tmp_nz) > 0:
    x_min = np.min(tmp_nz)
    x_max = np.max(tmp_nz)
  else:
    x_min = 0
    x_max = mask_image.shape[1]

  tmp_nz = np.where(imr.sum(axis=1) != 0)[0]
  if len(tmp_nz) > 0:
    y_min = np.min(tmp_nz)
    y_max = np.max(tmp_nz)
  else:
    y_min = 0
    y_max = mask_image.shape[0]

  if (x_max - x_min) == 0:
    x_min, x_max = 1, mask_image.shape[1] - 1
  else:
    tmp_d = (x_max - x_min) * 0.2
    x_min, x_max = x_min - tmp_d, x_max + tmp_d

  if (y_max - y_min) == 0:
    y_min, y_max = 1, mask_image.shape[0] - 1
  else:
    tmp_d = (y_max - y_min) * 0.2
    y_min, y_max = y_min - tmp_d, y_max + tmp_d

  # target image
  if ignore_mask:
    imt = img
  else:
    imt = img * imr

  if vis:
    plot_img("Masked image", imt, 1, vrange=[0.6, 1.0])

  # noise reduction
  if ns is not 0:
    imtm = binary_erosion(imt, structure=np.ones((ns, ns)).astype(np.int))
  else:
    imtm = np.where(imt > 0, 1, 0).astype(np.int)

  # image rotation
  imrot3 = pcrot(img, rot)
  imrot3 = imrot3 * np.where(img != 0, 1, 0)
  if vis:
    plot_img("Rot and thres (imrot3)", imrot3, 2, vrange=[0.95, 1.0])

  # reverse a rotated depth image
  img_rot = (np.max(imrot3) - imrot3) * np.where(imrot3 != 0, 1, 0)

  # target image for two-finger gripper
  if ignore_mask:
    imgt = img_rot
  else:
    imgt = img_rot * imr
  imgt = np.where(imgt != 0, 1, 0).astype(np.int)

  # debug
  #imgt = imtm

  # edge detection
  if parts_id not in {8, 14}:
    nmx, nmy, nmz = surfnorm(1000 * imrot3)
    imz = np.where(nmz > 0.9, 1, 0).astype(np.int)
    imgt = np.where(imgt * imz != 0, 1, 0)

  if vis:
    plot_img("N_z > 0.9 (imgt)", imgt, 8)
    plot_img("nmz", nmz, 3, vrange=[0.98, 1.0])

  # normal vectors
  nx, ny, nz = surfnorm(1000 * img)

  print("Elapsed time for preprocessing: {}".format(time() - t_start))

  # main processing
  if gripper_type is "suction":
    return _graspability_suction(
      img, imgt, imrot3, ml, hm, obj_size, nx, ny, nz, footprint_size, imtm,
      vis, x_min, x_max, y_min, y_max
    )
  elif gripper_type is "two_finger":
    return _graspability_two_finger(
      img, imgt, imrot3, img_rot, ml, hm, obj_size, d, nx, ny, nz,
      footprint_size, imtm, hm1, hm2, numo, vis, x_min, x_max, y_min, y_max
    )
  elif gripper_type is "inner":
    return _graspability_inner(
      img, imgt, img_rot, hm, nx, ny, nz, hough_radii, total_num_peaks,
      imrot3, vis, x_min, x_max, y_min, y_max
    )
  else:
    # TODO: raise error
    pass

def demo(parts_id, bin_id, gripper_type, ignore_mask, img_file,
         scale_depth, title, mask_image=None, vis=True):
  # NOTE: parts_id starts from 0, not 1, according to numpy indexing rule,
  #       which is different to MATLAB.
  from skimage import io

  img = io.imread(img_file) * scale_depth

  # set default mask image
  if mask_image is None:
    mask_file = "../../matlab_graspability/imr3_old.png"
    mask_image = imread(mask_file)

  # Run graspability
  t_start = time()
  result = graspability(img, parts_id=parts_id, bin_id=bin_id,
                        gripper_type=gripper_type, mask_image=mask_image,
                        vis=vis, footprint_size=50, ignore_mask=ignore_mask)
  print("Elapsed time = {}".format(time() - t_start))
  print("{} points found".format(len(result[0])))
  plt.suptitle(title)
  plt.show()

if __name__ == "__main__":
  """Example: python graspability.py 4 0 # parts_id is 5 (not 4)
  """
  import sys

  case_id = int(sys.argv[1])
  if len(sys.argv) >= 3:
    vis = False if str(sys.argv[2]) == "False" else True
  else:
    vis = True

  # Debug with Domae-san's data
  # if case_id is -2:
  #   from skimage import io
  #   import pandas as pd
  #
  #   data_dir = "/Users/taku-y/data/wrc2018/graspability/20181004"
  #   img_file = data_dir + "/sample.tif"
  #   mask_file = data_dir + "/imr3.png"
  #   gripper_type = "suction"
  #   img = io.imread(img_file) / 1000.0 # im meter
  #
  #   mask_img = io.imread(mask_file)
  #   parts_id = 4
  #   bin_id = 1
  #   result = graspability(img, parts_id=parts_id, bin_id=bin_id,
  #                         gripper_type=gripper_type, mask_image=mask_img,
  #                         vis=vis, footprint_size=50)
  #   df = pd.DataFrame(
  #     {
  #       "posx": result[0],
  #       "posy": result[1],
  #       "posz": result[2],
  #       "rotx": result[3],
  #       "roty": result[4],
  #       "rotz": result[5],
  #       "rotipz": result[6],
  #       "score": result[7]
  #     }
  #   )
  #   print(df)
  #   plt.show()
  #
  # elif case_id is -3:
  #   from skimage import io
  #   import pandas as pd
  #
  #   data_dir = "/Users/taku-y/data/wrc2018/graspability/20181004"
  #   img_file = data_dir + "/sample.tif"
  #   mask_file = data_dir + "/imr3.png"
  #   gripper_type = "inner"
  #   img = io.imread(img_file) / 1000.0  # im meter
  #
  #   row_min = 1100
  #   row_max = 1600
  #   col_min = 1100
  #   col_max = 1600
  #
  #   mask_img = io.imread(mask_file)
  #   parts_id = 7
  #   bin_id = 7
  #   result = graspability(img, parts_id=parts_id, bin_id=bin_id,
  #                         gripper_type=gripper_type, mask_image=mask_img,
  #                         vis=vis, footprint_size=50)
  #   df = pd.DataFrame(
  #     {
  #       "posx": result[0],
  #       "posy": result[1],
  #       "posz": result[2],
  #       "rotx": result[3],
  #       "roty": result[4],
  #       "rotz": result[5],
  #       "rotipz": result[6],
  #       "score": result[7]
  #     }
  #   )
  #   print(df)
  #   plt.show()

  if case_id is 7:
    from skimage import io
    import pandas as pd

    data_dir = "/Users/taku-y/data/wrc2018/graspability/20181004"
    img_file = data_dir + "/sample.tif"
    mask_file = data_dir + "/imr3.png"
    gripper_type = "suction"
    img = io.imread(img_file) / 1000.0 # im meter

    row_min = 500
    row_max = 1200
    col_min = 1200
    col_max = 1600

    mask_img = io.imread(mask_file)
    parts_id = 7
    bin_id = 2
    t_start = time()
    result = graspability(img, parts_id=parts_id, bin_id=bin_id,
                          gripper_type=gripper_type, mask_image=mask_img,
                          vis=vis, footprint_size=30)
    print("elapsed time: {}".format(time() - t_start))
    df = pd.DataFrame(
      {
        "posx": result[0],
        "posy": result[1],
        "posz": result[2],
        "rotx": result[3],
        "roty": result[4],
        "rotz": result[5],
        "rotipz": result[6],
        "score": result[7]
      }
    )
    print(df)

    image = result[8]

    if vis:
      plt.figure()
      plt.imshow(image)
      plt.show()

  elif case_id is 6:
    from skimage import io
    import pandas as pd

    data_dir = "/Users/taku-y/data/wrc2018/graspability/20181004"
    img_file = data_dir + "/sample.tif"
    mask_file = data_dir + "/imr3.png"
    gripper_type = "two_finger"
    img = io.imread(img_file) / 1000.0 # im meter
    row_min = 400
    row_max = 1200
    col_min = 0
    col_max = 600

    mask_img = io.imread(mask_file)
    parts_id = 6
    bin_id = 5
    t_start = time()
    result = graspability(img, parts_id=parts_id, bin_id=bin_id,
                          gripper_type=gripper_type, mask_image=mask_img,
                          vis=vis, footprint_size=50)
    print("elapsed time: {}".format(time() - t_start))
    df = pd.DataFrame(
      {
        "posx": result[0],
        "posy": result[1],
        "posz": result[2],
        "rotx": result[3],
        "roty": result[4],
        "rotz": result[5],
        "rotipz": result[6],
        "score": result[7]
      }
    )
    print(df)

    image = result[8]
    #print(image.shape, image.dtype)

    if vis:
      plt.figure()
      plt.imshow(image)
      plt.show()

  elif case_id is 17:
    from skimage import io
    import pandas as pd

    data_dir = "/Users/taku-y/data/wrc2018/graspability/20181004"
    img_file = data_dir + "/sample.tif"
    mask_file = data_dir + "/imr3.png"
    gripper_type = "inner"
    img = io.imread(img_file) / 1000.0 # im meter

    # row_min = 400
    # row_max = 1200
    # col_min = 0
    # col_max = 600

    mask_img = io.imread(mask_file)
    parts_id = 17
    bin_id = 0
    t_start = time()
    result = graspability(img, parts_id=parts_id, bin_id=bin_id,
                          gripper_type=gripper_type, mask_image=mask_img,
                          vis=vis, footprint_size=50)
    print("elapsed time: {}".format(time() - t_start))
    df = pd.DataFrame(
      {
        "posx": result[0],
        "posy": result[1],
        "posz": result[2],
        "rotx": result[3],
        "roty": result[4],
        "rotz": result[5],
        "rotipz": result[6],
        "score": result[7]
      }
    )
    print(df)

    image = result[8]
    #print(image.shape, image.dtype)

    if vis:
      plt.figure()
      plt.imshow(image)
      plt.show()
