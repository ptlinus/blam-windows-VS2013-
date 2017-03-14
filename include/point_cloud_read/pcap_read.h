#ifndef VLP16_READ_H
#define VLP16_READ_H
#include "winpcap/pcap/pcap.h"
#include<iostream>
#include<vector>

const int HDL_NUM_ROT_ANGLES = 36001;
const int HDL_LASER_PER_FIRING = 32;
const int HDL_MAX_NUM_LASERS = 64;
const int HDL_FIRING_PER_PKT = 12;
typedef long long vtkIdType;


enum HDLBlock
{
	BLOCK_0_TO_31 = 0xeeff,
	BLOCK_32_TO_63 = 0xddff
};

#pragma pack(push, 1)
typedef struct HDLLaserReturn
{
	unsigned short distance;
	unsigned char intensity;
} HDLLaserReturn;

struct HDLFiringData
{
	unsigned short blockIdentifier;
	unsigned short rotationalPosition;
	HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
};

struct HDLDataPacket
{
	HDLFiringData firingData[HDL_FIRING_PER_PKT];
	unsigned int gpsTimestamp;
	unsigned char blank1;
	unsigned char blank2;
};

struct HDLLaserCorrection
{
	double azimuthCorrection;
	double verticalCorrection;
	double distanceCorrection;
	double verticalOffsetCorrection;
	double horizontalOffsetCorrection;
	double sinVertCorrection;
	double cosVertCorrection;
	double sinVertOffsetCorrection;
	double cosVertOffsetCorrection;
};

struct HDLRGB
{
	unsigned char r;
	unsigned char g;
	unsigned char b;
};
#pragma pack(pop)


struct _array
{
	double xyz[3];
	double Intensity;
	int  LaserId;
	double Distance;
};
class MyHdlGrabber
{
public:
	MyHdlGrabber(const std::string& filename, const std::string& correctionsFile);
	~MyHdlGrabber();

public:
	bool GetFrame(int frameNumber);
	int ReadFrameInformation(void);
	int GetNumberOfFrames(void);
	void SetCorrectionsFile(const std::string& correctionsFile);
	void LoadCorrectionsFile(const std::string& correctionsFile);
	void SetFileName(const std::string& filename);
	void Init(const std::string& filename, const std::string& correctionsFile);

	void Close();
	void Open();

private:
	void InitTables();
	void ProcessHDLPacket(unsigned char *data, std::size_t bytesReceived);
	void SplitFrame(bool force);
	void ProcessFiring(HDLFiringData* firingData,
		int hdl64offset,
		int firingBlock,
		int azimuthDiff,
		double timestamp,
		unsigned int rawtime);
	void PushFiringData(const unsigned char laserId,
		const unsigned char rawLaserId,
		unsigned short azimuth,
		const double timestamp,
		const unsigned int rawtime,
		const HDLLaserReturn* laserReturn,
		const HDLLaserCorrection* correction,
		const bool dualReturn);
	double VLP16AdjustTimeStamp(int firingblock,
		int dsr,
		int firingwithinblock);
	double HDL32AdjustTimeStamp(int firingblock,
		int dsr);
	void UnloadData();

public:
	std::vector<_array>Points;

protected:
	std::vector<fpos_t> FilePositions;
	std::vector<int> Skips;


	int CalibrationReportedNumLasers;
	bool CorrectionsInitialized;

	// User configurable parameters
	int NumberOfTrailingFrames;
	int ApplyTransform;
	int PointsSkip;
	int SplitCounter;

	bool CropReturns;
	bool CropInside;
	double CropRegion[6];
	std::vector<double> cos_lookup_table_;
	std::vector<double> sin_lookup_table_;
	std::vector<bool> LaserSelection;
	unsigned int DualReturnFilter;

	std::string CorrectionsFile;
	std::string FileName;

	int Skip;
	bool endFrame = 0;
};

#endif