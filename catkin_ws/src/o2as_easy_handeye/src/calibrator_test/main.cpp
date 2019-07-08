#include <cstdlib>
#include <visp_bridge/3dpose.h>
#include <visp/vpHomogeneousMatrix.h>
#include "aistCalibration.h"

namespace aistCalibration
{
void
doJob(bool aist)
{
    size_t	nposes;
    std::cin >> nposes;

    std::vector<Transform>	cMo(nposes), wMe(nposes);
    for (size_t n = 0; n < nposes; ++n)
	std::cin >> cMo[n] >> wMe[n];

    Transform	eMc;
    if (aist)
    {
	eMc = calibrationAIST(cMo, wMe);
    }
    else
    {
	std::vector<vpHomogeneousMatrix>	cMo_vp;
	std::vector<vpHomogeneousMatrix>	wMe_vp;
	for (unsigned int i = 0; i < cMo.size(); i++)
	{
	    cMo_vp.push_back(visp_bridge::toVispHomogeneousMatrix(
				 geometry_msgs::Transform(cMo[i])));
	    wMe_vp.push_back(visp_bridge::toVispHomogeneousMatrix(
				 geometry_msgs::Transform(wMe[i])));
	}

	vpHomogeneousMatrix		eMc_vp;
	aistCalibration::calibrationTsai(cMo_vp, wMe_vp, eMc_vp);
	eMc = Transform(visp_bridge::toGeometryMsgsTransform(eMc_vp));
    }

    const auto	wMo = objectToWorld(cMo, wMe, eMc);
    evaluateAccuracy(std::cout, cMo, wMe, eMc, wMo);
}
    
}	// namespace aistCalibration

int
main(int argc, char* argv[])
{
    bool		aist = false;
    extern char*	optarg;
    for (int c; (c = getopt(argc, argv, "a")) !=EOF; )
	switch (c)
	{
	  case 'a':
	    aist = true;
	    break;
	}
    
    aistCalibration::doJob(aist);
    
}
