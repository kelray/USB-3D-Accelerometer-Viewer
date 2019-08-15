/*
 * Copyright 2018, 2019 Karim El-Rayes. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY HONG XU ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL HONG XU OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of Hong Xu.
 *
 *
 *
 *
 * Author: Karim El-Rayes
 *
 *
 * Project description: USB 3D Accelerometer using MMA8452Q 3-axis accelerometer and MCP2221A interface chip
 * Developer: Karim El-Rayes
 * Parts of the code related to interfacing the MMA8452Q 3-axis accelerometer over
 * I2C bus are derived from SparkFun MMA8452 Arduino library:
 * https://github.com/sparkfun/SparkFun_MMA8452Q_Arduino_Library
 * and "Arduino and MMA8452 sensor example" blog post listed here:
 * http://arduinolearning.com/code/arduino-and-mma8452-sensor-example.php
 * Dependencies:
 * MCP2221A C++ library and DLL version 2.1.1 from Microchip Technologies
 * Qt version 5.11 or higher
*/

//Qt header files
#include "mainwindow.h"
#include <QApplication>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDebug>
#include <QGraphicsItem>
#include <QGraphicsWidget>
#include <QLabel>
#include <QLineSeries>
#include <QtMath>
#include <QPushButton>
#include <QString>
#include <QSignalMapper>
#include <QTimer>
#include <QtCharts/QChartView>
#include <QtCharts/QValueAxis>
#include <QWidget>
#include <QMessageBox>
#include <QProgressBar>
#include <QSlider>
#include <QSpinBox>
#include <QComboBox>
#include <QTextEdit>
#include <QRadioButton>
#include <QMenu>
#include <QDial>
#include <QTabWidget>
#include <QMenuBar>
#include <QCheckBox>

//Standard C headers
#include <stdio.h>
#include <conio.h>
#include <Windows.h>
#include <iso646.h>

//External headers
#include "mcp2221_dll_um.h"

//To create a desktop app with all required Qt DLLs uncomment this line
//#pragma comment(linker, "/SUBSYSTEM:windows /ENTRY:mainCRTStartup")

#define pi 22/7
#define I2cAddr7bit 1
#define I2cAddr8bit 0

//Global variables
void *handle;
QString string;
QTextEdit *TextBox;
QTextEdit *ADCdataBox;
QVector<double> x(101), y(101);
int i;
int phase = 0;

//Chart variables
using namespace QtCharts;
QChartView *chartViewAcc;
QLineSeries *seriesX;
QLineSeries *seriesY;
QLineSeries *seriesZ;
QChart *chartAcc;
QPointF numX, numY, numZ;
QComboBox *RangeDropList;
QValueAxis *axisY1;

//checkboxes
QCheckBox *GPIO0chkBx;
QCheckBox *GPIO1chkBx;
QCheckBox *GPIO2chkBx;
QCheckBox *GPIO3chkBx;

int count = 0;
double xVal = 0, yVal = 0, zVal = 0;
int x1RangeMax = 180, x1RangeMin = 0, y1RangeMax = 2, y1RangeMin = -2;

char *token;
int RxBufferCounter = 0;
QString xValStr, yValStr, zValStr;
double AccVal[6];


wchar_t SerNum = 0x0000075428;
wchar_t LibVer[6];
wchar_t MfrDescriptor[30];
wchar_t ProdDescrip[30];
int ver = 0;
int error = 0;
int flag = 0;

unsigned int delay = 0;
unsigned int ReqCurrent;
unsigned int PID = 0xDD;
unsigned int VID = 0x4D8;
unsigned int NumOfDev = 0;
unsigned char PowerAttrib;

//I/O variables
unsigned char pinFunc[4] = {MCP2221_GPFUNC_IO, MCP2221_GPFUNC_IO, MCP2221_GPFUNC_IO, MCP2221_GPFUNC_IO};
unsigned char pinDir[4] = {MCP2221_GPDIR_OUTPUT, MCP2221_GPDIR_OUTPUT, MCP2221_GPDIR_OUTPUT, MCP2221_GPDIR_OUTPUT};
unsigned char OutValues[4] = {MCP2221_GPVAL_LOW, MCP2221_GPVAL_LOW, MCP2221_GPVAL_LOW, MCP2221_GPVAL_LOW};
unsigned int ADCbuffer[3];
unsigned char DacVal = 0;
unsigned int ADCreading = 0;
unsigned int NegativeFlag = 0;

//I2C variables
unsigned char SlaveAddress = 0x37;
unsigned char StartCmd = 0x00;
unsigned char RxData[7];
unsigned int tempADCreading = 0;
unsigned char TxData[2];

//MMA8452
unsigned char MMA8452_I2C_ADDR = 0x1C;			//MMA8452 accelerometer, SA0=0 Add=0x1C, SA0=1 Add=0x1D
unsigned char MMA8452_WHO_AM_I = 0x0D;
unsigned char MMA8452_CTRL_REG1 = 0x2A;
unsigned char MMA8452_XYZ_CFG_REG = 0x0E;
unsigned char MMA8452_STATUS_REG = 0x00;

//unsigned char setScale = 0;		//scale:
//unsigned char setODR = 0;		//output rate:
unsigned char setupTap[3] = {0};
enum MMA8452Q_Scale {SCALE_2G = 2, SCALE_4G = 4, SCALE_8G = 8}; // Possible full-scale settings
enum MMA8452Q_ODR {ODR_800, ODR_400, ODR_200, ODR_100, ODR_50, ODR_12, ODR_6, ODR_1}; // possible data rates
unsigned char config[2] = {0};
int FullScaleRange = SCALE_2G;  //default is +/-2g

//File logging variables
FILE * logFile;		//Create file to save temperature log

//Sine waveform LUT
unsigned int SineWave[] =  {0xe,0x10,0x13,0x15,0x17,0x19,0x1a,0x1b,
                            0x1b,0x1b,0x1a,0x19,0x17,0x15,0x13,0x10,
                            0xe,0xb,0x8,0x6,0x4,0x2,0x1,0x0,
                            0x0,0x0,0x1,0x2,0x4,0x6,0x8,0xb};

unsigned int ExpWave[] = {0x1c,0x1a,0x18,0x16,0x14,0x13,0x12,0x10,
                          0xf,0xe,0xd,0xc,0xb,0xa,0x9,0x9,
                          0x8,0x7,0x7,0x6,0x6,0x5,0x5,0x5,
                          0x4,0x4,0x4,0x3,0x3,0x3,0x3,0x2,
                          0x2,0x2,0x2,0x2,0x2,0x2,0x1,0x1,
                          0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x1,
                          0x1,0x1,0x1,0x1,0x0,0x0,0x0,0x0,
                          0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};

unsigned int TriWave[] = {0x2,0x3,0x5,0x7,0x8,0xa,0xc,0xe,
                          0xf,0x11,0x13,0x14,0x16,0x18,0x19,0x1b,
                          0x19,0x18,0x16,0x14,0x13,0x11,0xf,0xe,
                          0xc,0xa,0x8,0x7,0x5,0x3,0x2,0x0};
unsigned int LUT = 0;

void ExitFunc()
{
    //fclose(logFile);
    Mcp2221_Reset(handle);
}

unsigned int* MainWindow::ReadADC()
{
    flag = Mcp2221_GetAdcData(handle, ADCbuffer);
    if(flag != 0)
    {
        //printf("Error reading from ADC, error: %d\n", flag);
        string = "Error reading from ADC, error:" + QString::number(flag);
        TextBox->append(string);
    }
    else
    {
        string = "ADC channel 1 reading is: " + QString::number(ADCbuffer[0]) + "\nADC channel 2 reading is: " + QString::number(ADCbuffer[1]) + "\nADC channel 3 reading is: " + QString::number(ADCbuffer[2]);
        ADCdataBox->append(string);
    }
    return ADCbuffer;
}

void MainWindow::updateChart()
{
    //Read response from MMA8452
    flag = Mcp2221_I2cRead(handle, sizeof(RxData), MMA8452_I2C_ADDR, I2cAddr7bit, RxData);
    if(flag == 0)
    {

    }
    else
    {
        string = "Error reading from device: " + QString::number(flag);
        TextBox->append(string);
        Mcp2221_I2cCancelCurrentTransfer(handle);
    }

	// Convert the data to 12-bits
    int xAccl = ((RxData[1] * 256) + RxData[2]) / 16;
    if(xAccl > 2047)
    {
        xAccl -= 4096;
        //qDebug() << xAccl;
    }
    //printf("Acceleration in X: %d\n", xAccl);
    /*if(logFile != NULL)
    {
        fprintf(logFile,"%d,", xAccl);
    }*/

    int yAccl = ((RxData[3] * 256) + RxData[4]) / 16;
    if(yAccl > 2047)
    {
        yAccl -= 4096;
    }
    //printf("Acceleration in Y: %d\n", yAccl);
    /*if(logFile != NULL)
    {
        fprintf(logFile,"%d, ", yAccl);
    }*/

    int zAccl = ((RxData[5] * 256) + RxData[6]) / 16;
    if(zAccl > 2047)
    {
        zAccl -= 4096;
    }

    if(logFile != NULL)
    {
        fprintf(logFile,"%d,", xAccl);
        fprintf(logFile,"%d, ", yAccl);
        fprintf(logFile,"%d,\n", zAccl);
    }
       
    AccVal[0] = (float)xAccl / (float)(1 << 11) * (float)(FullScaleRange);
    AccVal[1] = (float)yAccl / (float)(1 << 11) * (float)(FullScaleRange);
    AccVal[2] = (float)zAccl / (float)(1 << 11) * (float)(FullScaleRange);

    count++;
    if(count == x1RangeMax)
    {
        count = 0;
        seriesX->clear();
        seriesY->clear();
        seriesZ->clear();
    }
    numX.setX(count);
    numX.setY(AccVal[0]);

    numY.setX(count);
    numY.setY(AccVal[1]);

    numZ.setX(count);
    numZ.setY(AccVal[2]);

    seriesX->append(numX);
    seriesY->append(numY);
    seriesZ->append(numZ);
    
}

void Mcp2221_config()
{
    ver = Mcp2221_GetLibraryVersion(LibVer);		//Get DLL version
    if(ver == 0)
    {
        //printf("Library (DLL) version: %ls\n", LibVer);
        string = "Library (DLL) version: "+QString::fromWCharArray(LibVer);
        TextBox->append(string);
    }
    else
    {
        error = Mcp2221_GetLastError();
        //printf("Version can't be found, version: %d, error: %d\n", ver, error);
        string = "Cannot get version, error: " + error;
        TextBox->append(string);
    }

    //Get number of connected devices with this VID & PID
    Mcp2221_GetConnectedDevices(VID, PID, &NumOfDev);
    if(NumOfDev == 0)
    {
        //printf("No MCP2221 devices connected\n");
        string = "No MCP2221 devices connected";
        TextBox->append(string);
        //exit(0);
    }
    else
    {
        //printf("Number of devices found: %d\n", NumOfDev);
        string = "Number of devices found: " + QString::number(NumOfDev);
        TextBox->append(string);
    }

    //open device by S/N
    //handle = Mcp2221_OpenBySN(VID, PID, &SerNum);

    //Open device by index
    handle = Mcp2221_OpenByIndex(VID, PID, NumOfDev-1);
    if(error == NULL)
    {
        //printf("Connection successful\n");
        string = "Connection successful";
        TextBox->append(string);
    }
    else
    {
        error = Mcp2221_GetLastError();
        //printf("Error message is %s\n", error);
        string = "Error message is "+ QString::number(error);
        TextBox->append(string);
    }

    //Get manufacturer descriptor
    flag = Mcp2221_GetManufacturerDescriptor(handle, MfrDescriptor);
    if(flag == 0)
    {
        //printf("Manufacturer descriptor: %ls\n", MfrDescriptor);
        string = "Manufacturer descriptor: " + QString::fromWCharArray(MfrDescriptor);
        TextBox->append(string);
    }
    else
    {
        //printf("Error getting descriptor: %d\n", flag);
        string = "Error getting descriptor: " + QString::number(flag);
        TextBox->append(string);
    }

    //Get product descriptor
    flag = Mcp2221_GetProductDescriptor(handle, ProdDescrip);
    if(flag == 0)
    {
        //printf("Product descriptor: %ls\n", ProdDescrip);
        string = "Product descriptor: " + QString::fromWCharArray(ProdDescrip);
        TextBox->append(string);
    }
    else
    {
        //printf("Error getting product descriptor: %d\n", flag);
        string = "Error getting product descriptor:" + QString::number(flag);
        TextBox->append(string);
    }

    //Get power attributes
    flag = Mcp2221_GetUsbPowerAttributes(handle, &PowerAttrib, &ReqCurrent);
    if(flag == 0)
    {
        //printf("Power Attributes, %x\nRequested current units = %d\nRequested current(mA) = %d\n", PowerAttrib, ReqCurrent, ReqCurrent*2);
        string = "Power Attributes " + QString::number(PowerAttrib) + "\nRequested current units = " + QString::number(ReqCurrent) + "\nRequested current(mA) = " + QString::number(ReqCurrent*2);
        TextBox->append(string);
    }
    else
    {
        //printf("Error getting power attributes: %d\n", flag);
        string = "Error getting power attributes:"+ QString::number(flag);
        TextBox->append(string);
    }

    //Set I2C bus
    flag = Mcp2221_SetSpeed(handle, 500000);    //set I2C speed to 400 KHz
    if(flag == 0)
    {
        //printf("I2C is configured\n");
        string = "I2C is configured";
        TextBox->append(string);
    }
    else
    {
        //printf("Error setting I2C bus: %d\n", flag);
        string = "Error setting I2C bus:"+ QString::number(flag);
        TextBox->append(string);
    }

    //Set I2C advanced parameters
    flag = Mcp2221_SetAdvancedCommParams(handle, 10, 100);  //10ms timeout, try 1000 times
    if(flag == 0)
    {
        //printf("I2C advanced settings set\n");
        string = "I2C advanced settings set";
        TextBox->append(string);
    }
    else
    {
        //printf("Error setting I2C advanced settings: %d\n", flag);
        string = "Error setting I2C advanced settings:"+ QString::number(flag);
        TextBox->append(string);
    }

    //Set GPIO
    flag = Mcp2221_SetGpioSettings(handle, RUNTIME_SETTINGS, pinFunc, pinDir, OutValues);
    if(flag != 0)
    {
        //printf("Error setting GPIO, error: %d\n", flag);
        string = "Error setting GPIO, error: "+ QString::number(flag);
        TextBox->append(string);
    }
}

void MainWindow::DropBoxCallback(int index)
{
    switch(RangeDropList->currentIndex())
    {
        case 0:
            qDebug() << "droplist index: " << RangeDropList->currentIndex();
            config[1] = 0x00;	//2g:0x00, 4g: 0x01, 8g: 0x02
            y1RangeMax = 2;
            y1RangeMin = -2;
            string = "Setting MMA8452 range to: +/- 2g ";
            FullScaleRange = SCALE_2G;
            qDebug() << "droplist index: " << RangeDropList->currentIndex() << "value: " << config[1];
            break;

        case 1:
            config[1] = 0x01;	//2g:0x00, 4g: 0x01, 8g: 0x02
            y1RangeMax = 4;
            y1RangeMin = -4;
            string = "Setting MMA8452 range to: +/- 4g ";
            FullScaleRange = SCALE_4G;
            qDebug() << "droplist index: " << RangeDropList->currentIndex() << "value: " << config[1];
            break;

        case 2:
            config[1] = 0x02;	//2g:0x00, 4g: 0x01, 8g: 0x02
            y1RangeMax = 8;
            y1RangeMin = -8;
            string = "Setting MMA8452 range to: +/- 8g ";
            FullScaleRange = SCALE_8G;
            qDebug() << "droplist index: " << RangeDropList->currentIndex() << "value: " << config[1];
            break;
    }
    config[0] = MMA8452_XYZ_CFG_REG;
    axisY1->setRange(y1RangeMin, y1RangeMax);

    flag = Mcp2221_I2cWrite(handle, sizeof(config), MMA8452_I2C_ADDR, I2cAddr7bit, config);    //issue start condition then address
    if(flag == 0)
    {
        //string = "Setting MMA8452 range to: " + QString::number(MMA8452_I2C_ADDR);
        TextBox->append(string);
    }
    else
    {
        string = "Error writing to device: " + QString::number(flag);
        TextBox->append(string);
        Mcp2221_I2cCancelCurrentTransfer(handle);
    }
}

void MMA8452_Config()
{
    //Get MMA8452 device ID
    flag = Mcp2221_I2cWrite(handle, sizeof(MMA8452_WHO_AM_I), MMA8452_I2C_ADDR, I2cAddr7bit, &MMA8452_WHO_AM_I);
    if(flag == 0)
    {
        string = "Writing to device: " + QString::number(MMA8452_I2C_ADDR) + " successful";
        TextBox->append(string);
    }
    else
    {
        string = "Error requesting device ID: " + QString::number(flag);
        TextBox->append(string);
        Mcp2221_I2cCancelCurrentTransfer(handle);
    }

    //Read response from MMA8452
    flag = Mcp2221_I2cRead(handle, 1, MMA8452_I2C_ADDR, I2cAddr7bit, RxData);	//sizeof(RxData)
    if(flag == 0)
    {
        //string = "Device ID is: " + QString::number(RxData[0]);
        string = "Device ID is: 0x" + QString::number(RxData[0], 16).toUpper();
        TextBox->append(string);
    }
    else
    {
        string = "Error reading device ID: " + QString::number(flag);
        TextBox->append(string);
        Mcp2221_I2cCancelCurrentTransfer(handle);
    }

    // Active mode(0x03)
    //0b00000011: 50Hz auto-wake rate (00), 800 Hz ODR (000), reduced noise mode disabled (0), fast read mode enabled (1), active mode (1)
    //config[0] = 0x2A;
    config[0] = MMA8452_CTRL_REG1;
    config[1] = 0x01;//0x03: fast read enables, data sample is limited to one byte (8-bits per sample)

    flag = Mcp2221_I2cWrite(handle, sizeof(config), MMA8452_I2C_ADDR, I2cAddr7bit, config);    //issue start condition then address
    if(flag == 0)
    {
        string = "Setting MMA8452 rate: " + QString::number(MMA8452_I2C_ADDR);
        TextBox->append(string);
    }
    else
    {
        string = "Error writing configuration to device: " + QString::number(flag);
        TextBox->append(string);
        Mcp2221_I2cCancelCurrentTransfer(handle);
    }

    // Select configuration register(0x0E)
    // Set range to +/- 2g(0x00)
    //config[0] = 0x0E;
    config[0] = MMA8452_XYZ_CFG_REG;
    config[1] = 0x00;	//2g:0x00, 4g: 0x01, 8g: 0x02

    flag = Mcp2221_I2cWrite(handle, sizeof(config), MMA8452_I2C_ADDR, I2cAddr7bit, config);    //issue start condition then address
    if(flag == 0)
    {
        string = "Setting MMA8452 range: " + QString::number(MMA8452_I2C_ADDR);
        TextBox->append(string);
    }
    else
    {
        string = "Error writing to device: " + QString::number(flag);
        TextBox->append(string);
        Mcp2221_I2cCancelCurrentTransfer(handle);
    }

    // Read 7 bytes of data(0x00)
    // staus, xAccl msb, xAccl lsb, yAccl msb, yAccl lsb, zAccl msb, zAccl lsb
    //unsigned char reg[1] = {0x00};

    flag = Mcp2221_I2cWrite(handle, sizeof(MMA8452_STATUS_REG), MMA8452_I2C_ADDR, I2cAddr7bit, &MMA8452_STATUS_REG);    //issue start condition then address
    if(flag == 0)
    {
        string = "Requesting data from MMA8452 device: " + QString::number(MMA8452_I2C_ADDR);
        TextBox->append(string);
    }
    else
    {
        string = "Error writing to device: " + QString::number(flag);
        TextBox->append(string);
        Mcp2221_I2cCancelCurrentTransfer(handle);
    }
}

void MainWindow::ChkBxCallback(int index)
{
    if(GPIO0chkBx->isChecked())
        OutValues[0] = 1;
    else
        OutValues[0] = 0;

    if(GPIO1chkBx->isChecked())
        OutValues[1] = 1;
    else
        OutValues[1] = 0;

    if(GPIO2chkBx->isChecked())
        OutValues[2] = 1;
    else
        OutValues[2] = 0;

    if(GPIO3chkBx->isChecked())
        OutValues[3] = 1;
    else
        OutValues[3] = 0;
    Mcp2221_SetGpioValues(handle, OutValues);
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setFixedSize(650, 680);
    w.setWindowTitle("USB 3D Accelerometer Viewer");   //Window title

    atexit(ExitFunc);	//Call exit function

    //Label 1 - MCP2221 info
    QLabel *Label1 = new QLabel(&w);
    Label1->setGeometry(15, 460, 150, 25);
    Label1->setText("Device Configuration info");

    //Textbox for device information
    TextBox = new QTextEdit(&w);
    TextBox->setGeometry(15, 485, 380, 150);
    TextBox->acceptRichText();
    TextBox->setAcceptRichText(true);

    //MMA8452 range
    QLabel *RangeLabel = new QLabel(&w);
    RangeLabel->setGeometry(410, 460, 180, 25);
    RangeLabel->setText("Select MMA8452 Range:");

    //Drop-down list
    RangeDropList = new QComboBox(&w);
    RangeDropList->setGeometry(410, 485, 100, 35);
    RangeDropList->addItem("2g");
    RangeDropList->addItem("4g");
    RangeDropList->addItem("8g");


    //checkbox for GPIO - output only
    QSignalMapper *CheckBoxSignalMapper = new QSignalMapper(&w);
    GPIO0chkBx = new QCheckBox("GPIO0",&w);
    GPIO0chkBx->setGeometry(410, 535, 100, 20);
    GPIO1chkBx = new QCheckBox("GPIO1",&w);
    GPIO1chkBx->setGeometry(410, 565, 100, 20);
    GPIO2chkBx = new QCheckBox("GPIO2",&w);
    GPIO2chkBx->setGeometry(410, 595, 100, 20);
    GPIO3chkBx = new QCheckBox("GPIO3",&w);
    GPIO3chkBx->setGeometry(410, 625, 100, 20);

    //Create line series for X,Y,Z
    seriesX = new QLineSeries();
    seriesX->setName("X-axis");
    seriesX->setColor(QColor::QColor("Red"));
    seriesY = new QLineSeries();
    seriesY->setName("Y-axis");
    seriesY->setColor(QColor::QColor("Blue"));
    seriesZ = new QLineSeries();
    seriesZ->setName("Z-axis");
    seriesZ->setColor(QColor::QColor("Green"));

    //Accelerometer Chart
    chartAcc = new QChart();
    chartAcc->setGeometry(15, 15, 620, 450);
    chartAcc->legend()->hide();
    chartAcc->addSeries(seriesX);
    chartAcc->addSeries(seriesY);
    chartAcc->addSeries(seriesZ);
    chartAcc->createDefaultAxes();
    chartAcc->setTitle("Simple line chart example");
    chartAcc->setPlotAreaBackgroundVisible(true);

    //Create Accelerometer Chart View Widget
    chartViewAcc = new QChartView(chartAcc, &w);
    chartViewAcc->setGeometry(15, 15, 620, 450);

    //Acelerometer char axes
    QValueAxis *axisX1 = new QValueAxis;
    axisX1->setRange(x1RangeMin, x1RangeMax);
    axisX1->setTitleText("Time");

    //QValueAxis *axisY1 = new QValueAxis;
    axisY1 = new QValueAxis;
    axisY1->setRange(y1RangeMin, y1RangeMax);
    axisY1->setTitleText("Acceleration (g)");

    //Add series to Accelerometer char
    chartAcc->setBackgroundVisible(false);
    chartAcc->setAxisX(axisX1, seriesX);
    chartAcc->setAxisY(axisY1, seriesX);
    chartAcc->setAxisX(axisX1, seriesY);
    chartAcc->setAxisY(axisY1, seriesY);
    chartAcc->setAxisX(axisX1, seriesZ);
    chartAcc->setAxisY(axisY1, seriesZ);
    chartAcc->legend()->show();
    chartAcc->setTitle("Accelerometer");

    //create a timer and connect it to real-time slot
    QTimer *timer = new QTimer(&w);
    QObject::connect(timer, SIGNAL(timeout()), &w, SLOT(updateChart()));
    timer->start(0);    //Interval 0 means to refresh as fast as possible

    //Create file to save log with todays date & time
    //ogFile = fopen("logFile.csv","w+");
    //fprintf(logFile, "X, Y, Z,\n");

    //Configure MCP2221 & MMA88452
    Mcp2221_config();
    MMA8452_Config();

    w.show();
    QObject::connect(RangeDropList, SIGNAL(currentIndexChanged(int)), &w, SLOT(DropBoxCallback(int)));

    QObject::connect(CheckBoxSignalMapper, SIGNAL(mapped(int)), &w, SLOT(ChkBxCallback(int)));
    CheckBoxSignalMapper->setMapping(GPIO0chkBx, 0);
    CheckBoxSignalMapper->setMapping(GPIO1chkBx, 1);
    CheckBoxSignalMapper->setMapping(GPIO2chkBx, 2);
    CheckBoxSignalMapper->setMapping(GPIO3chkBx, 3);
    QObject::connect(GPIO0chkBx, SIGNAL(stateChanged(int)), CheckBoxSignalMapper, SLOT(map()));
    QObject::connect(GPIO1chkBx, SIGNAL(stateChanged(int)), CheckBoxSignalMapper, SLOT(map()));
    QObject::connect(GPIO2chkBx, SIGNAL(stateChanged(int)), CheckBoxSignalMapper, SLOT(map()));
    QObject::connect(GPIO3chkBx, SIGNAL(stateChanged(int)), CheckBoxSignalMapper, SLOT(map()));

    return a.exec();
}
