/*!
 * Author: Gu Qiyang guqiyang@aicrobo.com
 * Group: AICRobo http://aicrobo.github.io
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, AICRobo.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <boost/regex.hpp>
#include <bitset>
#include "rovioParser.h"

using namespace std;

void rovioParser::initCurl()
{
  curl = curl_easy_init();
  if (curl == NULL)
  {
    ROS_ERROR("Curl initialize failed.");
    exit(-1);
  }
}

void rovioParser::initParam(std::string host, std::string port, std::string user, std::string pw)
{
  host="192.168.10.18";
  user="admin";
  pw="123456";
  curl_easy_setopt(curl, CURLOPT_USERPWD, (user + ":" + pw).c_str());
  this->url = (boost::format("http://%s:%s") % host % port).str();
}

rovioParser::rovioParser()
{
  initCurl();
}

rovioParser::rovioParser(std::string host, std::string port, std::string user, std::string pw)
{
  initCurl();
  initParam(host, port, user, pw);
}

rovioParser::~rovioParser()
{
  curl_easy_cleanup(curl);
}

rvCR writeDataCR(void *ptr, size_t size, size_t nmemb, void *stream)
{
  rvCR rv;

  return rv;
}

size_t rovioParser::WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
  ((std::string*)userp)->append((char*)contents, size * nmemb);
  return size * nmemb;
}

rvManualDrive rovioParser::manualDrive(int dValue, int sValue)
{
  rvManualDrive rv;
  string resp;
  curl_easy_setopt(curl, CURLOPT_URL, (url + (boost::format("/rev.cgi?Cmd=nav&action=18&drive=%d&speed=%d") % dValue % sValue).str()).c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resp);
  curl_easy_perform(curl);
//
//  boost::regex expression("Cmd = (\\w+).*\\n*.*responses = (\\d+)");
//  boost::smatch what;
//  boost::regex_search(resp, what, expression);
//
//  rv.cmd = what[1];
//  rv.responses = boost::lexical_cast<int>(what[2]);
  return rv;
}

cv::Mat rovioParser::getImg()
{
  FILE *fp;
  fp = fopen("temp.jpg", "w");
  curl_easy_setopt(curl, CURLOPT_URL, (url + "/Jpeg/CamImg.jpg").c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, NULL);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
  curl_easy_perform(curl);
  fclose(fp);

  return cv::imread("temp.jpg");
}

rvMCUReport rovioParser::getMCUReport()
{
  rvMCUReport rv;
  string resp;
  curl_easy_setopt(curl, CURLOPT_URL, (url + "/rev.cgi?Cmd=nav&action=20").c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resp);
  curl_easy_perform(curl);

  boost::regex expression("Cmd = (\\w+).*\\n*.*responses = (\\w+)");
  boost::smatch what;
  boost::regex_search(resp, what, expression);

  for (int i = 0; i < what.size(); i++)
  {
    std::cout << i << ": " << what[i] << endl;
  }

  rv.cmd = what[1];
  string mcu = what[2];
  if (30 == mcu.size())
  {
    bitset<16> bitVec;

    rv.length = strtol(mcu.substr(0, 2).c_str(), NULL, 16);

    bitVec = strtol(mcu.substr(4, 2).c_str(), NULL, 16);
    if (!bitVec[0] && !bitVec[1])
      rv.lDirection = 0;
    else if (bitVec[0])
      rv.lDirection = 1;
    else if (bitVec[1])
      rv.lDirection = -1;
    else
      ROS_ERROR("The info of left wheel direction is wrong!");
    rv.lNum = strtol(mcu.substr(6, 4).c_str(), NULL, 16);

    bitVec = strtol(mcu.substr(10, 2).c_str(), NULL, 16);
    if (!bitVec[0] && !bitVec[1])
      rv.rDirection = 0;
    else if (bitVec[0])
      rv.rDirection = 1;
    else if (bitVec[1])
      rv.rDirection = -1;
    else
      ROS_ERROR("The info of right wheel direction is wrong!");
    rv.rNum = strtol(mcu.substr(12, 4).c_str(), NULL, 16);

    bitVec = strtol(mcu.substr(16, 2).c_str(), NULL, 16);
    if (!bitVec[0] && !bitVec[1])
      rv.rearDirection = 0;
    else if (bitVec[0])
      rv.rearDirection = -1;
    else if (bitVec[1])
      rv.rearDirection = 1;
    else
      ROS_ERROR("The info of rear wheel direction is wrong!");
    rv.rearNum = strtol(mcu.substr(18, 4).c_str(), NULL, 16);

    rv.headPosition = strtol(mcu.substr(24, 2).c_str(), NULL, 16);

    bitVec = strtol(mcu.substr(28, 2).c_str(), NULL, 16);
    rv.isLedOn = bitVec[0];
    rv.isIrOn = bitVec[1];
    rv.isDetectedBarrier = bitVec[2];
  }
  else
    ROS_ERROR("Fetch MCU info Error!");

  printMCUReport(rv);

  return rv;
}

void rovioParser::printMCUReport(rvMCUReport report)
{
  cout << "Cmd = " << report.cmd << endl;
  cout << "length = " << report.length << endl;
  cout << "lDirection = " << report.lDirection << endl;
  cout << "lNum = " << report.lNum << endl;
  cout << "rDirection = " << report.rDirection << endl;
  cout << "rNum = " << report.rNum << endl;
  cout << "rearDirection = " << report.rearDirection << endl;
  cout << "rearNum = " << report.rearNum << endl;
  cout << "headPosition = " << report.headPosition << endl;
  cout << "batteryStatus = " << report.batteryStatus << endl;
  cout << "isLedOn = " << report.isLedOn << endl;
  cout << "isIrOn = " << report.isIrOn << endl;
  cout << "isDetectedBarrier = " << report.isDetectedBarrier << endl;
  cout << "chargerStatus = " << report.chargerStatus << endl;
}

void rovioParser::test()
{
  string a = "E3123456789";
  uint value;
  value = strtol(a.substr(0, 2).c_str(), NULL, 16);

  uchar uc;
  uint ui;
  char c;
  int i;
  bitset<8> bitvec;
  stringstream ss("05");
  ss >> hex >> ui;
  bitvec = ui;

  char k = bitvec[0];
  ss >> c;
  {
    unsigned int a = 0;
    string s = "F010";
    stringstream ss(s);
    ss >> hex >> a;
    cout << a << endl;
    ss >> c;
  }
}
