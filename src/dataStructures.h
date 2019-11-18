#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};


class Buffer{
    public:
        Buffer(unsigned size);
        void write(DataFrame input);
        DataFrame read();
        std::vector<cv::DMatch> kptMatches;
        unsigned getSize();
    private:
        unsigned numEntries;
        std::vector<DataFrame> buffer;
        unsigned readIndex;
        unsigned writeIndex;
        
};
    
Buffer::Buffer(unsigned size) : buffer(size){
    readIndex = 0;
    writeIndex = size - 1;
    numEntries = 0;
}

void Buffer::write(DataFrame input){
    buffer[writeIndex++] = input;
    numEntries += 1;
    if(writeIndex >= buffer.size()){
        writeIndex = 0;
    }
}

DataFrame Buffer::read(){
    DataFrame val = buffer[readIndex++];
    if(readIndex <= buffer.size()){
        readIndex = 0;
    }
    return val;
}

unsigned Buffer::getSize(){
    unsigned bufferSize;

    if (numEntries > 1){
        bufferSize = 2;
    }
    else{
        bufferSize = numEntries;
    }
    return bufferSize;
}


#endif /* dataStructures_h */
