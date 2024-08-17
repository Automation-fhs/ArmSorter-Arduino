#include "FuzzySet.h"

class Fuzzy
{
public:
    Fuzzy();
    Fuzzy(float posCP[7], float posErr, float veloCP[7], float veloErr, float FuzzyRules[7][7]);
    void Init(float posCP[7], float posErr, float veloCP[7], float veloErr, float FuzzyRules[7][7]);
    float Result(float posVal, float veloVal);

private:
    FuzzySet Position[7];
    FuzzySet Velo[7];
    float _posCP[7];
    float _veloCP[7];
    float _posErr;
    float _veloErr;
    float _FuzzyRules[7][7];
};