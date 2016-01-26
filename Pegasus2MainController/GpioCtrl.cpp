#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include "GpioCtrl.h"

using namespace std;

GpioCtrl::GpioCtrl()
{
    gpionum = "4"; //GPIO4 is default
}

GpioCtrl::GpioCtrl(string gnum)
{
    gpionum = gnum;  //Instatiate GpioCtrl object for GPIO pin number "gnum"
}

int GpioCtrl::export_gpio()
{
    string export_str = "/sys/class/gpio/export";
    ofstream exportgpio(export_str.c_str()); // Open "export" file. Convert C++ string to C string. Required for all Linux pathnames
    if (exportgpio < 0){
        cout << " OPERATION FAILED: Unable to export GPIO" << gpionum << " ." << endl;
        return -1;
    }

    exportgpio << gpionum; //write GPIO number to export
    exportgpio.close(); //close export file
    return 0;
}

int GpioCtrl::unexport_gpio()
{
    string unexport_str = "/sys/class/gpio/unexport";
    ofstream unexportgpio(unexport_str.c_str()); //Open unexport file
    if (unexportgpio < 0){
        cout << " OPERATION FAILED: Unable to unexport GPIO" << gpionum << " ." << endl;
        return -1;
    }

    unexportgpio << gpionum; //write GPIO number to unexport
    unexportgpio.close(); //close unexport file
    return 0;
}

int GpioCtrl::setdir_gpio(string dir)
{

    string setdir_str = "/sys/class/gpio/gpio" + gpionum + "/direction";
    ofstream setdirgpio(setdir_str.c_str()); // open direction file for gpio
    if (setdirgpio < 0){
        cout << " OPERATION FAILED: Unable to set direction of GPIO" << gpionum << " ." << endl;
        return -1;
    }

    setdirgpio << dir; //write direction to direction file
    setdirgpio.close(); // close direction file
    return 0;
}

int GpioCtrl::setval_gpio(string val)
{

    string setval_str = "/sys/class/gpio/gpio" + gpionum + "/value";
    ofstream setvalgpio(setval_str.c_str()); // open value file for gpio
    if (setvalgpio < 0){
        cout << " OPERATION FAILED: Unable to set the value of GPIO" << gpionum << " ." << endl;
        return -1;
    }

    setvalgpio << val;//write value to value file
    setvalgpio.close();// close value file
    return 0;
}

int GpioCtrl::getval_gpio(string& val){

    string getval_str = "/sys/class/gpio/gpio" + gpionum + "/value";
    ifstream getvalgpio(getval_str.c_str());// open value file for gpio
    if (getvalgpio < 0){
        cout << " OPERATION FAILED: Unable to get value of GPIO" << gpionum << " ." << endl;
        return -1;
    }

    getvalgpio >> val;  //read gpio value

    if (val != "0")
        val = "1";
    else
        val = "0";

    getvalgpio.close(); //close the value file
    return 0;
}

string GpioCtrl::get_gpionum(){

    return gpionum;

}