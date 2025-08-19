#ifndef POSITION_CALCULATOR_H
#define POSITION_CALCULATOR_H

constexpr int tolerance = 10;

class PositionCalculator {
public:
    PositionCalculator(float minLength = 0.0, float maxLength = 130.0)
        : minAdc_(0), maxAdc_(4095), minLength_(minLength), maxLength_(maxLength), isInverse_(false) {}

    void setAdcLimits(int minAdc, int maxAdc) {
        if (isInverse_){
            maxAdc = minAdc;
            minAdc = maxAdc;
        }
        else
        {
            minAdc_ = minAdc;
            maxAdc_ = maxAdc;
        }
    }

    void setAdcMinLimit(int minAdc) {
        if (isInverse_) {
            maxAdc_ = minAdc;
        } else {
            minAdc_ = minAdc;
        }
    }

    void setAdcMaxLimit(int maxAdc) {
        if (isInverse_) {
            minAdc_ = maxAdc;
        } else {
            maxAdc_ = maxAdc;
        }
    }

    void setLengthLimits(float minLength, float maxLength) {
        minLength_ = minLength;
        maxLength_ = maxLength;
    }

    float computePosition(int adcValue) const {
        if (adcValue < minAdc_) adcValue = minAdc_;
        if (adcValue > maxAdc_) adcValue = maxAdc_;

        float normalized = (float)(adcValue - minAdc_) / (maxAdc_ - minAdc_);
        float position = minLength_ + normalized * (maxLength_ - minLength_);
        
        if (isInverse_) {
            position = map(position, 0, 130, 130, 0);
        }
        return position;
    }

    bool minLimitReached(int adcValue) const {
        if(isInverse_){
            return adcValue >= maxAdc_ - tolerance;
        }
        return adcValue <= minAdc_ + tolerance;  
    }

    bool maxLimitReached(int adcValue) const {
        if(isInverse_){
            return adcValue <= minAdc_ + tolerance;
        }
        return adcValue >= maxAdc_ - tolerance;
    }

    int minLimit() const {
        if (isInverse_) {
            return maxAdc_;
        }
        return minAdc_;
    }

    int maxLimit() const {
        if (isInverse_) {
            return minAdc_;
        }
        return maxAdc_;
    }

    void setInverse(bool inverse) {
        isInverse_ = inverse;
    }

private:
    bool isInverse_ = false;
    int minAdc_;
    int maxAdc_; 
    float minLength_;
    float maxLength_;
};

#endif
