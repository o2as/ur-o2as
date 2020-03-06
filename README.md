# O2AS UR Project

`master`: [![pipeline status](https://gitlab.com/o2as/ur-o2as/badges/master/pipeline.svg)](https://gitlab.com/o2as/ur-o2as/commits/master) `devel`: [![pipeline status](https://gitlab.com/o2as/ur-o2as/badges/devel/pipeline.svg)](https://gitlab.com/o2as/ur-o2as/commits/devel) 

This project stores the software running the robot solution of Team O2AS in the Industrial Challenge of the [World Robot Summit 2018](http://worldrobotsummit.org/en/). Installation is fully automated, and the solution runs inside a Docker container. Only Docker is installed on the host PC (Ubuntu 16.04 or newer).

## QUICK START

1) Clone the repository into your home folder in Ubuntu 16.04 (or newer).  
2) Execute the three scripts (SETUP-DEVEL-MACHINE.sh, BUILD-DOCKER-IMAGE.sh, RUN-DOCKER-CONTAINER.sh)  
3) Read the documentation and report here if you have any problems.  

## Getting Started

All the necessary documentation and step-by-step guides to get started can be found in the GitLab wiki:  
https://github.com/o2as/ur-o2as/wikis/home

## Contribution Guidelines

As the project is being deployed for the first time, minor compatibility issues are expected to arise. Please report any issue. Please carefully read the contribution guidelines before pushing code or requesting a merge. Details can be found in the GitLab wiki:
https://github.com/o2as/ur-o2as/wikis/contribution-guidelines

## Credits

Please cite our paper ([arXiv](http://arxiv.org/abs/2003.02427)) if this code has been insightful for your research or development:  

BibTeX:  
```
@article{doi:10.1080/01691864.2020.1734481,
author = {von Drigalski,Felix and Nakashima,Chisato and Shibata,Yoshiya and Konishi,Yoshinori and Triyonoputro,Joshua and Nie,Kaidi and Petit,Damien and Ueshiba,Toshio and Takase,Ryuichi and Domae,Yukiyasu and Yoshioka,Taku and Ijiri,Yoshihisa and Ramirez-Alpizar,Ixchel Georgina and Wan,Weiwei and Harada,Kensuke},
title = {Team O2AS at the World Robot Summit 2018: An Approach to Robotic Kitting and Assembly Tasks using General Purpose Grippers and Tools},
journal = {Advanced Robotics},
year = {2020},
doi = {10.1080/01691864.2020.1734481},
}
```

This project is based on the HSR environment maintained at the [Emergent Systems Laboratory](http://www.em.ci.ritsumei.ac.jp/), Department of Human and Computer Intelligence, College of Information Science and Engineering, [Ritsumeikan University](http://en.ritsumei.ac.jp/).
