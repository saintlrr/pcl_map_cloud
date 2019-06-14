#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include <numeric>

template <typename T>
std::vector<size_t> sort_index(const std::vector<T> &v)
{
    std::vector<size_t> idx_list(v.size());
    std::iota(idx_list.begin(), idx_list.end(), 0);
    sort(idx_list.begin(), idx_list.end(), [&v](size_t i1, size_t i2)
        {return v[i1] < v[i2];});
    return idx_list;
}

#endif