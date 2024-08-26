class FuzzySet
{
public:
    void Init(float a1, float a2, float d1, float d2)
    {
        this->_a1 = a1;
        this->_a2 = a2;
        this->_d1 = d1;
        this->_d2 = d2;
    }
    float membership(float val)
    {
        if (val <= this->_a1)
            return 0;
        if (val >= this->_d2)
            return 0;
        if (val >= this->_a2 && val <= this->_d1)
            return 1;
        if (val > this->_a1 && val < this->_a2)
            return (val - this->_a1) / (this->_a2 - this->_a1);
        if (val > this->_d1 && val < this->_d2)
            return 1 - (val - this->_d1) / (this->_d2 - this->_d1);
    }
    bool checkValue(float val) {
        if(val >= this->_a1 && val <= this->_d2) return true;
        else return false;
    };
private:
    float _a1, _a2, _d1, _d2;
};