#ifndef POSITION_CALCULATOR_H
#define POSITION_CALCULATOR_H

constexpr int tolerance = 10;

class PositionCalculator {
public:
    PositionCalculator(float minLength = 0.0, float maxLength = 130.0)
        : minAdc_(0), maxAdc_(4095), minLength_(minLength), maxLength_(maxLength) {}

    void setAdcLimits(int minAdc, int maxAdc) {
        minAdc_ = minAdc;
        maxAdc_ = maxAdc;
    }

    void setAdcMinLimit(int minAdc) {
        minAdc_ = minAdc;
    }

    void setAdcMaxLimit(int maxAdc) {
        maxAdc_ = maxAdc;
    }

    void setLengthLimits(float minLength, float maxLength) {
        minLength_ = minLength;
        maxLength_ = maxLength;
    }

    float computePosition(int adcValue) const {
        if (adcValue < minAdc_) adcValue = minAdc_;
        if (adcValue > maxAdc_) adcValue = maxAdc_;

        float normalized = (float)(adcValue - minAdc_) / (maxAdc_ - minAdc_);
        return minLength_ + normalized * (maxLength_ - minLength_);
    }

    bool minLimitReached(int adcValue) const {
        return adcValue <= minAdc_ + tolerance;
    }

    bool maxLimitReached(int adcValue) const {
        return adcValue >= maxAdc_ - tolerance;
    }

private:
    int minAdc_;
    int maxAdc_; 
    float minLength_;
    float maxLength_;
};

#endif
