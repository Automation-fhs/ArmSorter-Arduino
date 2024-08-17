#include "Fuzzy.h"

Fuzzy::Fuzzy() {};

Fuzzy::Fuzzy(float posCP[7], float posErr, float veloCP[7], float veloErr, float FuzzyRules[7][7])
{
    for (int i = 0; i < 7; i++)
    {
        this->_posCP[i] = posCP[i];
        this->_veloCP[i] = veloCP[i];
        for (int j = 0; j < 7; j++)
        {
            this->_FuzzyRules[i][j] = FuzzyRules[i][j];
        }
    }
    Position[0].Init(this->_posCP[0], this->_posCP[0], this->_posCP[0], this->_posCP[1]);
    Position[1].Init(this->_posCP[0], this->_posCP[1] - this->_posErr, this->_posCP[1] + this->_posErr, this->_posCP[2]);
    Position[2].Init(this->_posCP[1], this->_posCP[2], this->_posCP[2], this->_posCP[3]);
    Position[3].Init(this->_posCP[2], this->_posCP[3] - this->_posErr, this->_posCP[3] + this->_posErr, this->_posCP[4]);
    Position[4].Init(this->_posCP[3], this->_posCP[4], this->_posCP[4], this->_posCP[5]);
    Position[5].Init(this->_posCP[4], this->_posCP[5] - this->_posErr, this->_posCP[5] + this->_posErr, this->_posCP[6]);
    Position[6].Init(this->_posCP[5], this->_posCP[6], this->_posCP[6], this->_posCP[6]);

    Velo[0].Init(this->_veloCP[0], this->_veloCP[0], this->_veloCP[0], this->_veloCP[1]);
    Velo[1].Init(this->_veloCP[0], this->_veloCP[1], this->_veloCP[1], this->_veloCP[2]);
    Velo[2].Init(this->_veloCP[1], this->_veloCP[2], this->_veloCP[2], this->_veloCP[3]);
    Velo[3].Init(this->_veloCP[2], this->_veloCP[3] - this->_veloErr, this->_veloCP[3] + this->_veloErr, this->_veloCP[4]);
    Velo[4].Init(this->_veloCP[3], this->_veloCP[4], this->_veloCP[4], this->_veloCP[5]);
    Velo[5].Init(this->_veloCP[4], this->_veloCP[5], this->_veloCP[5], this->_veloCP[6]);
    Velo[6].Init(this->_veloCP[5], this->_veloCP[6], this->_veloCP[6], this->_veloCP[6]);
}

void Fuzzy::Init(float posCP[7], float posErr, float veloCP[7], float veloErr, float FuzzyRules[7][7])
{
    for (int i = 0; i < 7; i++)
    {
        this->_posCP[i] = posCP[i];
        this->_veloCP[i] = veloCP[i];
        for (int j = 0; j < 7; j++)
        {
            this->_FuzzyRules[i][j] = FuzzyRules[i][j];
        }
    }
    Position[0].Init(this->_posCP[0], this->_posCP[0], this->_posCP[0], this->_posCP[1]);
    Position[1].Init(this->_posCP[0], this->_posCP[1] - this->_posErr, this->_posCP[1] + this->_posErr, this->_posCP[2]);
    Position[2].Init(this->_posCP[1], this->_posCP[2], this->_posCP[2], this->_posCP[3]);
    Position[3].Init(this->_posCP[2], this->_posCP[3] - this->_posErr, this->_posCP[3] + this->_posErr, this->_posCP[4]);
    Position[4].Init(this->_posCP[3], this->_posCP[4], this->_posCP[4], this->_posCP[5]);
    Position[5].Init(this->_posCP[4], this->_posCP[5] - this->_posErr, this->_posCP[5] + this->_posErr, this->_posCP[6]);
    Position[6].Init(this->_posCP[5], this->_posCP[6], this->_posCP[6], this->_posCP[6]);

    Velo[0].Init(this->_veloCP[0], this->_veloCP[0], this->_veloCP[0], this->_veloCP[1]);
    Velo[1].Init(this->_veloCP[0], this->_veloCP[1], this->_veloCP[1], this->_veloCP[2]);
    Velo[2].Init(this->_veloCP[1], this->_veloCP[2], this->_veloCP[2], this->_veloCP[3]);
    Velo[3].Init(this->_veloCP[2], this->_veloCP[3] - this->_veloErr, this->_veloCP[3] + this->_veloErr, this->_veloCP[4]);
    Velo[4].Init(this->_veloCP[3], this->_veloCP[4], this->_veloCP[4], this->_veloCP[5]);
    Velo[5].Init(this->_veloCP[4], this->_veloCP[5], this->_veloCP[5], this->_veloCP[6]);
    Velo[6].Init(this->_veloCP[5], this->_veloCP[6], this->_veloCP[6], this->_veloCP[6]);
}

float min(float a, float b)
{
    if (a < b)
        return a;
    else
        return b;
}

float Fuzzy::Result(float posVal, float veloVal)
{
    float sumVal = 0;
    float sumMem = 0;
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            sumVal += this->_FuzzyRules[i][j] * min(Position[i].membership(posVal), Velo[j].membership(veloVal));
            // console.log(`Value at pos ${i} vel ${j} is ${Math.min(Position[i].membership(posVal),Velo[j].membership(veloVal))}`);
            sumMem += min(Position[i].membership(posVal), Velo[j].membership(veloVal));
        }
    }
    return sumVal / sumMem;
}