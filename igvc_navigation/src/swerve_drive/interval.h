/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Mark Naeem
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * The names of the contributors may NOT be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Mark Naeem
 */


#include <vector>
#include <array>
#include <string>
#include <iostream>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include "utils.h"

class interval{
    std::vector<std::array<double,2>> limits_;
    std::vector<std::array<int,2>> types_;
    std::unordered_map<std::string,std::array<int,2>> types_dict_ = { {"open", std::array<int,2>({0,0}) }, {"closeopen", std::array<int,2>({1,0}) }, {"openclose", std::array<int,2>({0,1}) }, {"close", std::array<int,2>({1,1}) } };
    
    std::vector<utils::limits_types_intervals> limits_types_intervals_vector_;

    bool intersect( std::array<double,2>& limits1, const std::array<double,2>& limits2, std::array<int,2>& type1, const std::array<int,2>& type2);
    
    public:
        interval(const std::array<double,2>& limits = {-3.14, 3.14}, const std::string& type="close");
        bool is_intersecting(const interval& another_interval);
        interval complement() const;
        double len() const;
        void print() const;
}; 