# SFND Camera Midterm Project

## MP.1 Data Buffer

In the dataStructures.h file, a class called Buffer was created. This buffer holds a vector of Dataframes, and has read and write methods. Calling write will push entries in the end, and calling read will pull entries out of the front.

## MP.2 Keypoint Detection

In matching2D_Student.cpp, one function was added, detKeypointsHarris, which followed the approach from the lessons. A second functions detKeypointsModern was added which calls on the various modern detectors using a string selector, detectorType, to choose which detector to select. Note that an additional input was added to all these functions, called output, so that outputs could be tracked in files.

## MP.3 Keypoint Removal

Around line 87 of MidtermProject_Camera_Student.cpp, the input images were cropped prior to passing into the detectors. First, this approach seemed much simpler to me than removing keypoints outside of some box, and second if this the only region which will be analyzed, that it seems like wasted overhead to run on the entire image, this would make everything much slower.

## MP.4 Keypoint Descriptors

In matching2D_Student.cpp, the function descKeypoints was expanded to included requested descriptors. Note a few things had to be massaged here to avoid errors. First, it seems that the SIFT descriptor class needs to CV_32 images to operate on, so a catch to transform images before passing to that descriptor is added. Second, from reading online, it seems that AKZE descriptors can only be used with AKAZE keypoints, so if this descriptor is passed it will just recompute keypoints as AKAZE and post a notification. Timing and output tracking handling also added.

## MP.5, MP.6 Descriptor Matching and KNN

In matching2D_Student.cpp, the function matchDescriptors was added. KNN and FLANN were added. Note that BF matching was modified relative to lessons since SIFT needs an actual distance based on floats, so L2 is used for all cases for simplicity.

KNN is also added following the example in the lessons, using a descriptor distance ratio cutoff of 0.8 as directed for comparing descriptors.

## MP.7, MP.8, MP.9 Comparing Performance

To run different combos, added a command line argument which allows one to select a descriptor and detector like this:

2D_feature_tracking "HARRIS" "SIFT"

Where the first arg is the detector and second is the descriptor. The output files for all of the possible combinations are placed in outputs directory.

For aggregating, the analyze.ipynb is added (since my python is much strongly than my C++ :), this was much faster for me). This pulls everything together into summary.csv, so nothing of relevance to this assignment is in the python file.

Looking at summary.csv, it is clear that extracting keypoints is the major bottleneck, and that FAST works as advertised in the name. In general it also seems that FAST detects quite a few keypoints on the vehicles (second most to SHITOMASI, at ~100), and the number of matches are significant for most of the detector types (~15-60). Given the spped up, it seems this is a good detector choice overall.

FAST along with ORB and BRIEF seem to offer fewer matches than BRISK and FREAK, and SIFT is patented, so I shall never it use it again after this class. Probably I would choose FAST and FREAK to start with, about 30 matches with a run time <ms seems like good performance to first order.
