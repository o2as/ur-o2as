/*!
 *   \file	BinDescription.h
 */
#include <iostream>
#include <tf/transform_datatypes.h>

namespace o2as
{
class BinDescription
{
  private:
    static tf::Vector3
    getRPY(const tf::Quaternion& q)
    {
	const tf::Matrix3x3	rot(q);
	double			roll, pitch, yaw;
	rot.getRPY(roll, pitch, yaw);
	return {roll, pitch, yaw};
    }
    
  public:
    BinDescription(int part_id, const tf::Transform& transform)
	:_set_id(1), _bin_id(1), _bin_num(0), _part_id(part_id),
	 _xyz(transform.getOrigin()), _rpy(getRPY(transform.getRotation())),
	 _z_origin_offset(0)
    {
	tfScalar	marker_height = 0;
	
	switch (_part_id)
	{
	  case 9:	// End cap
	  case 10:	// Spacer(output)
	  case 12:	// Spacer(tension)
	  case 14:	// Retainer pin
	  case 15:	// M6 nut
	  case 16:	// M6 Washer
	  case 17:	// M4 Bol
	  case 18:	// M3 Bolt
	    _bin_id = 1;
	    _bin_num = ++_nbins[_bin_id - 1];
	    _z_origin_offset = -0.008;
	    marker_height = 0.010;
	    break;
	    
	  case 4:	// Geared motor
	  case 5:	// Pulley 30mm
	  case 7:	// Bearings
	  case 8:	// Drive shaft
	  case 11:	// Pully 60mm
	    _bin_id = 2;
	    _bin_num = ++_nbins[_bin_id - 1];
	    _z_origin_offset = -0.009;
	    marker_height = 0.010;
	    break;
	    
	  case 6:	// Round belt
	    _bin_id = 3;
	    _bin_num = ++_nbins[_bin_id - 1];
	    _z_origin_offset = -0.004;
	    marker_height = 0.010 + 0.073;
	    break;
	    
	  default:
	    break;
	}

	_xyz.setZ(_xyz.z() - marker_height);
    }

    static void
    clear()
    {
	std::fill(std::begin(_nbins), std::begin(_nbins), 0);
    }
    

    std::ostream&
    print_pose(std::ostream& out) const
    {
	constexpr tfScalar	degree = 180.0/M_PI;
	
	return out << "  <xacro::kitting_bin_" << _bin_id
		   << " binname=\"set" << _set_id
		   << "_bin" << _bin_id
		   << '_' << _bin_num
		   << "\" parent=\"workspace_center\" z_origin_offset=\""
		   << _z_origin_offset
		   << "\">\n    <origin xyz=\""
		   << _xyz.x() << ' ' << _xyz.y() << ' ' << _xyz.z()
		   << "\" rpy=\""
		   <<  "$(" << _rpy.x()*degree << "*pi/180)"
		   << " $(" << _rpy.y()*degree << "*pi/180)"
		   << " $(" << _rpy.z()*degree << "*pi/180)\"/>\n"
		   << "  </xacro::kitting_bin_" << _bin_id << ">";
    }

    std::ostream&
    print_part(std::ostream& out) const
    {
	return out << "part_" << _part_id
		   << ",bin"  << _bin_id
		   << ",set"  << _set_id
		   << "_bin"  << _bin_id
		   << '_'     << _bin_num;
    }
    
    friend std::ostream&
    operator <<(std::ostream& out, const BinDescription& bin)
    {
	return bin.print_pose(out);
    }

  private:
    const int		_set_id;
    int			_bin_id;
    int			_bin_num;
    const int		_part_id;
    tf::Vector3		_xyz;
    const tf::Vector3	_rpy;
    tfScalar		_z_origin_offset;

    static std::array<int, 3>	_nbins;	// _nbins[0] = #bin1,...
};

}	// namespace o2as
