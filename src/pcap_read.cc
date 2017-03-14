#include <point_cloud_read/pcap_read.h>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>
#ifdef _MSC_VER
# include <boost/cstdint.hpp>
typedef boost::uint8_t uint8_t;
#else
# include <stdint.h>
#endif
// Some versions of libpcap do not have PCAP_NETMASK_UNKNOWN
#if !defined(PCAP_NETMASK_UNKNOWN)
#define PCAP_NETMASK_UNKNOWN 0xffffffff
#endif
static double Pi() { return 3.141592653589793; };

#define HDL_Grabber_toRadians(x) ((x) * Pi() / 180.0)

MyHdlGrabber::MyHdlGrabber(const std::string& filename, const std::string& correctionsFile){
	Init(filename, correctionsFile);
	ReadFrameInformation();
	Open();

}
MyHdlGrabber::~MyHdlGrabber(){
	Close();
}


class vtkPacketFileReader
{
public:

	vtkPacketFileReader()
	{
		this->PCAPFile = 0;
	}

	~vtkPacketFileReader()
	{
		this->Close();
	}

	bool Open(const std::string& filename)
	{
		char errbuff[PCAP_ERRBUF_SIZE];
		pcap_t *pcapFile = pcap_open_offline(filename.c_str(), errbuff);
		if (!pcapFile)
		{
			this->LastError = errbuff;
			return false;
		}

		struct bpf_program filter;

		if (pcap_compile(pcapFile, &filter, "udp", 0, PCAP_NETMASK_UNKNOWN) == -1)
		{
			this->LastError = pcap_geterr(pcapFile);
			return false;
		}

		if (pcap_setfilter(pcapFile, &filter) == -1)
		{
			this->LastError = pcap_geterr(pcapFile);
			return false;
		}

		this->FileName = filename;
		this->PCAPFile = pcapFile;
		this->StartTime.tv_sec = this->StartTime.tv_usec = 0;
		return true;
	}

	bool IsOpen()
	{
		return (this->PCAPFile != 0);
	}

	void Close()
	{
		if (this->PCAPFile)
		{
			pcap_close(this->PCAPFile);
			this->PCAPFile = 0;
			this->FileName.clear();
		}
	}

	const std::string& GetLastError()
	{
		return this->LastError;
	}

	const std::string& GetFileName()
	{
		return this->FileName;
	}

	void GetFilePosition(fpos_t* position)
	{
#ifdef _MSC_VER
		pcap_fgetpos(this->PCAPFile, position);
#else
		FILE* f = pcap_file(this->PCAPFile);
		fgetpos(f, position);
#endif
	}

	void SetFilePosition(fpos_t* position)
	{
#ifdef _MSC_VER
		pcap_fsetpos(this->PCAPFile, position);
#else
		FILE* f = pcap_file(this->PCAPFile);
		fsetpos(f, position);
#endif
	}

	bool NextPacket(const unsigned char*& data, unsigned int& dataLength, double& timeSinceStart, pcap_pkthdr** headerReference = NULL)
	{
		if (!this->PCAPFile)
		{
			return false;
		}

		struct pcap_pkthdr *header;
		int returnValue = pcap_next_ex(this->PCAPFile, &header, &data);
		if (returnValue < 0)
		{
			this->Close();
			return false;
		}

		if (headerReference != NULL)
		{
			*headerReference = header;
			dataLength = header->len;
			timeSinceStart = GetElapsedTime(header->ts, this->StartTime);
			return true;
		}

		// The ethernet header is 42 bytes long; unnecessary
		const unsigned int bytesToSkip = 42;
		dataLength = header->len - bytesToSkip;
		data = data + bytesToSkip;
		timeSinceStart = GetElapsedTime(header->ts, this->StartTime);
		return true;
	}

protected:

	double GetElapsedTime(const struct timeval& end, const struct timeval& start)
	{
		return (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.00;
	}

	pcap_t* PCAPFile;
	std::string FileName;
	std::string LastError;
	struct timeval StartTime;
};



#define UINT_MAX      0xffffffff    /* maximum unsigned int value */

bool IsDualReturnData;
bool IsHDL64Data;
int LastAzimuth;
unsigned int LastTimestamp;
double TimeAdjust;
vtkIdType LastPointId[HDL_MAX_NUM_LASERS];
vtkIdType FirstPointIdThisReturn;
vtkPacketFileReader* Reader;
HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];



enum DualFlag {
	DUAL_DISTANCE_NEAR = 0x1,   // point with lesser distance
	DUAL_DISTANCE_FAR = 0x2,    // point with greater distance
	DUAL_INTENSITY_HIGH = 0x4,  // point with lesser intensity
	DUAL_INTENSITY_LOW = 0x8,   // point with greater intensity
	DUAL_DOUBLED = 0xf,         // point is single return
	DUAL_DISTANCE_MASK = 0x3,
	DUAL_INTENSITY_MASK = 0xc,
};




//-----------------------------------------------------------------------------
void MyHdlGrabber::InitTables(void)
{
	if (cos_lookup_table_.size() == 0 || sin_lookup_table_.size() == 0)
	{
		cos_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
		sin_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
		for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++)
		{
			double rad = HDL_Grabber_toRadians(i / 100.0);
			cos_lookup_table_[i] = std::cos(rad);
			sin_lookup_table_[i] = std::sin(rad);
		}
	}
}

void MyHdlGrabber::Init(const std::string& filename, const std::string& correctionsFile)
{
	Skip = 0;
	LastAzimuth = -1;
	LastTimestamp = UINT_MAX;// std::numeric_limits<unsigned int>::max();
	TimeAdjust = std::numeric_limits<double>::quiet_NaN();
	Reader = 0;
	SplitCounter = 0;
	NumberOfTrailingFrames = 0;
	ApplyTransform = 0;
	PointsSkip = 0;
	CropReturns = false;
	CropInside = false;
	CropRegion[0] = CropRegion[1] = 0.0;
	CropRegion[2] = CropRegion[3] = 0.0;
	CropRegion[4] = CropRegion[5] = 0.0;
	CorrectionsInitialized = false;

	std::fill(LastPointId, LastPointId + HDL_MAX_NUM_LASERS, -1);

	LaserSelection.resize(64, true);
	DualReturnFilter = 0;
	IsDualReturnData = false;
	IsHDL64Data = false;

	InitTables();

	SetFileName(filename);
	SetCorrectionsFile(correctionsFile);
}

//-----------------------------------------------------------------------------
void MyHdlGrabber::UnloadData()
{
	std::fill(LastPointId, LastPointId + HDL_MAX_NUM_LASERS, -1);
	LastAzimuth = -1;
	LastTimestamp = UINT_MAX;// std::numeric_limits<unsigned int>::max();
	TimeAdjust = std::numeric_limits<double>::quiet_NaN();

	IsDualReturnData = false;
	IsHDL64Data = false;
	//Datasets.clear();
	//CurrentDataset = this->Internal->CreateData(0);
}


void MyHdlGrabber::SetFileName(const std::string& filename)
{
	if (filename == FileName)
	{
		return;
	}

	FileName = filename;
	FilePositions.clear();
	Skips.clear();
	UnloadData();
	//this->Modified();
}



//-----------------------------------------------------------------------------
void MyHdlGrabber::LoadCorrectionsFile(const std::string& correctionsFile)
{
	boost::property_tree::ptree pt;
	try
	{
		read_xml(correctionsFile, pt, boost::property_tree::xml_parser::trim_whitespace);
	}
	catch (boost::exception const&)
	{
		return;
	}

	int enabledCount = 0;
	BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("boost_serialization.DB.enabled_"))
	{
		std::stringstream ss;
		if (v.first == "item")
		{
			ss << v.second.data();
			int test = 0;
			ss >> test;
			if (!ss.fail() && test == 1)
			{
				enabledCount++;
			}
		}
	}
	CalibrationReportedNumLasers = enabledCount;

	BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("boost_serialization.DB.points_"))
	{
		if (v.first == "item")
		{
			boost::property_tree::ptree points = v.second;
			BOOST_FOREACH(boost::property_tree::ptree::value_type &px, points)
			{
				if (px.first == "px")
				{
					boost::property_tree::ptree calibrationData = px.second;
					int index = -1;
					double azimuth = 0;
					double vertCorrection = 0;
					double distCorrection = 0;
					double vertOffsetCorrection = 0;
					double horizOffsetCorrection = 0;

					BOOST_FOREACH(boost::property_tree::ptree::value_type &item, calibrationData)
					{
						if (item.first == "id_")
							index = atoi(item.second.data().c_str());
						if (item.first == "rotCorrection_")
							azimuth = atof(item.second.data().c_str());
						if (item.first == "vertCorrection_")
							vertCorrection = atof(item.second.data().c_str());
						if (item.first == "distCorrection_")
							distCorrection = atof(item.second.data().c_str());
						if (item.first == "vertOffsetCorrection_")
							vertOffsetCorrection = atof(item.second.data().c_str());
						if (item.first == "horizOffsetCorrection_")
							horizOffsetCorrection = atof(item.second.data().c_str());
					}
					if (index != -1)
					{
						laser_corrections_[index].azimuthCorrection = azimuth;
						laser_corrections_[index].verticalCorrection = vertCorrection;
						laser_corrections_[index].distanceCorrection = distCorrection / 100.0;
						laser_corrections_[index].verticalOffsetCorrection = vertOffsetCorrection / 100.0;
						laser_corrections_[index].horizontalOffsetCorrection = horizOffsetCorrection / 100.0;

						laser_corrections_[index].cosVertCorrection = std::cos(HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
						laser_corrections_[index].sinVertCorrection = std::sin(HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
					}
				}
			}
		}
	}

	for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
	{
		HDLLaserCorrection correction = laser_corrections_[i];
		laser_corrections_[i].sinVertOffsetCorrection = correction.verticalOffsetCorrection
			* correction.sinVertCorrection;
		laser_corrections_[i].cosVertOffsetCorrection = correction.verticalOffsetCorrection
			* correction.cosVertCorrection;
	}
	CorrectionsInitialized = true;
}

//-----------------------------------------------------------------------------
void MyHdlGrabber::SetCorrectionsFile(const std::string& correctionsFile)
{
	if (correctionsFile == CorrectionsFile)
	{
		return;
	}

	if (!boost::filesystem::exists(correctionsFile) ||
		boost::filesystem::is_directory(correctionsFile))
	{

		return;
	}
	LoadCorrectionsFile(correctionsFile);

	CorrectionsFile = correctionsFile;
	UnloadData();
	//this->Modified();
}


int MyHdlGrabber::GetNumberOfFrames(void)
{
	return FilePositions.size();;
}


//-----------------------------------------------------------------------------
int MyHdlGrabber::ReadFrameInformation(void)
{
	vtkPacketFileReader reader;
	if (!reader.Open(FileName))
	{
		return 0;
	}

	const unsigned char* data = 0;
	unsigned int dataLength = 0;
	double timeSinceStart = 0;

	unsigned int lastAzimuth = 0;
	unsigned int lastTimestamp = 0;

	std::vector<fpos_t> filePositions;
	std::vector<int> skips;

	fpos_t lastFilePosition;
	reader.GetFilePosition(&lastFilePosition);


	filePositions.push_back(lastFilePosition);
	skips.push_back(0);

	while (reader.NextPacket(data, dataLength, timeSinceStart))
	{

		if (dataLength != 1206)
		{
			continue;
		}

		const HDLDataPacket* dataPacket = reinterpret_cast<const HDLDataPacket *>(data);

		//    unsigned int timeDiff = dataPacket->gpsTimestamp - lastTimestamp;
		//    if (timeDiff > 600 && lastTimestamp != 0)
		//      {
		//      printf("missed %d packets\n",  static_cast<int>(floor((timeDiff/553.0) + 0.5)));
		//      }

		for (int i = 0; i < HDL_FIRING_PER_PKT; ++i)
		{
			HDLFiringData firingData = dataPacket->firingData[i];

			if (firingData.rotationalPosition < lastAzimuth)
			{
				filePositions.push_back(lastFilePosition);
				skips.push_back(i);
				//UpdateProgress(0.0);
			}

			lastAzimuth = firingData.rotationalPosition;
		}

		lastTimestamp = dataPacket->gpsTimestamp;
		reader.GetFilePosition(&lastFilePosition);
	}

	FilePositions = filePositions;
	Skips = skips;
	return GetNumberOfFrames();
}




static int Round(float f) {
	return static_cast<int>(f + (f >= 0 ? 0.5 : -0.5));
}
static int Round(double f) {
	return static_cast<int>(f + (f >= 0 ? 0.5 : -0.5));
}


//-----------------------------------------------------------------------------
double MyHdlGrabber::HDL32AdjustTimeStamp(int firingblock,
	int dsr)
{
	return (firingblock * 46.08) + (dsr * 1.152);
}

//-----------------------------------------------------------------------------
double MyHdlGrabber::VLP16AdjustTimeStamp(int firingblock,
	int dsr,
	int firingwithinblock)
{
	return (firingblock * 110.592) + (dsr * 2.304) + (firingwithinblock * 55.296);
}


//-----------------------------------------------------------------------------
void MyHdlGrabber::PushFiringData(const unsigned char laserId,
	const unsigned char rawLaserId,
	unsigned short azimuth,
	const double timestamp,
	const unsigned int rawtime,
	const HDLLaserReturn* laserReturn,
	const HDLLaserCorrection* correction,
	const bool dualReturn)
{
	azimuth %= 36000;
	static vtkIdType cnt = 0;
	const vtkIdType thisPointId = Points.size();// this->Points->GetNumberOfPoints();
	const short intensity = laserReturn->intensity;

	double cosAzimuth, sinAzimuth;
	if (correction->azimuthCorrection == 0)
	{
		cosAzimuth = cos_lookup_table_[azimuth];
		sinAzimuth = sin_lookup_table_[azimuth];
	}
	else
	{
		double azimuthInRadians = HDL_Grabber_toRadians((static_cast<double> (azimuth) / 100.0) - correction->azimuthCorrection);
		cosAzimuth = std::cos(azimuthInRadians);
		sinAzimuth = std::sin(azimuthInRadians);
	}

	double distanceM = laserReturn->distance * 0.002 + correction->distanceCorrection;
	double xyDistance = distanceM * correction->cosVertCorrection;

	// Compute raw position
	double pos[3] =
	{
		xyDistance * sinAzimuth - correction->horizontalOffsetCorrection * cosAzimuth,
		xyDistance * cosAzimuth + correction->horizontalOffsetCorrection * sinAzimuth,
		distanceM * correction->sinVertCorrection + correction->verticalOffsetCorrection
	};

	// Apply sensor transform
	// this->SensorTransform->InternalTransformPoint(pos, pos);

	// Test if point is cropped
	if (CropReturns)
	{
		bool pointOutsideOfBox = pos[0] >= CropRegion[0] && pos[0] <= CropRegion[1] &&
			pos[1] >= CropRegion[2] && pos[1] <= CropRegion[3] &&
			pos[2] >= CropRegion[4] && pos[2] <= CropRegion[5];
		if ((pointOutsideOfBox && !CropInside) ||
			(!pointOutsideOfBox && CropInside))
		{
			return;
		}
	}

	// Do not add any data before here as this might short-circuit
	if (dualReturn)
	{
		const vtkIdType dualPointId = LastPointId[rawLaserId];
		if (dualPointId < FirstPointIdThisReturn)
		{
			// No matching point from first set (skipped?)
			//Flags->InsertNextValue(DUAL_DOUBLED);
			//DistanceFlag->InsertNextValue(0);
			//IntensityFlag->InsertNextValue(0);
		}
		else
		{
			const short dualIntensity = 0;// this->Intensity->GetValue(dualPointId);
			const double dualDistance = 0;// this->Distance->GetValue(dualPointId);
			unsigned int firstFlags = 0;// this->Flags->GetValue(dualPointId);
			unsigned int secondFlags = 0;

			if (dualDistance == distanceM && intensity == dualIntensity)
			{
				// ignore duplicate point and leave first with original flags
				return;
			}

			if (dualIntensity < intensity)
			{
				firstFlags &= ~DUAL_INTENSITY_HIGH;
				secondFlags |= DUAL_INTENSITY_HIGH;
			}
			else
			{
				firstFlags &= ~DUAL_INTENSITY_LOW;
				secondFlags |= DUAL_INTENSITY_LOW;
			}

			if (dualDistance < distanceM)
			{
				firstFlags &= ~DUAL_DISTANCE_FAR;
				secondFlags |= DUAL_DISTANCE_FAR;
			}
			else
			{
				firstFlags &= ~DUAL_DISTANCE_NEAR;
				secondFlags |= DUAL_DISTANCE_NEAR;
			}

			// We will output only one point so return out of this
			if (DualReturnFilter)
			{
				if (!(secondFlags & DualReturnFilter))
				{
					return;
				}
				if (!(firstFlags & DualReturnFilter))
				{
					return;
				}
			}
		}
	}
	else
	{
	}
	_array dat;
	dat.xyz[0] = pos[0];
	dat.xyz[1] = pos[1];
	dat.xyz[2] = pos[2];
	dat.LaserId = laserId;
	dat.Intensity = laserReturn->intensity;
	dat.Distance = distanceM;

	Points.push_back(dat);
	//this->Azimuth->InsertNextValue(azimuth);
	//this->Intensity->InsertNextValue(laserReturn->intensity);
	//this->LaserId->InsertNextValue(laserId);
	//this->Timestamp->InsertNextValue(timestamp);
	//this->RawTime->InsertNextValue(rawtime);
	//this->Distance->InsertNextValue(distanceM);
	//this->LastPointId[rawLaserId] = thisPointId;
}
//-----------------------------------------------------------------------------
void MyHdlGrabber::ProcessFiring(HDLFiringData* firingData,
	int hdl64offset,
	int firingBlock,
	int azimuthDiff,
	double timestamp,
	unsigned int rawtime)
{
	const bool dual = (LastAzimuth == firingData->rotationalPosition) &&
		(!IsHDL64Data);

	if (!dual)
	{
		//FirstPointIdThisReturn = Points->GetNumberOfPoints();
	}

	if (dual && !IsDualReturnData)
	{
		IsDualReturnData = true;
		//	this->CurrentDataset->GetPointData()->AddArray(this->DistanceFlag.GetPointer());
		//	this->CurrentDataset->GetPointData()->AddArray(this->IntensityFlag.GetPointer());
	}

	for (int dsr = 0; dsr < HDL_LASER_PER_FIRING; dsr++)
	{
		unsigned char rawLaserId = static_cast<unsigned char>(dsr + hdl64offset);
		unsigned char laserId = rawLaserId;
		unsigned short azimuth = firingData->rotationalPosition;

		// Detect VLP-16 data and adjust laser id if necessary
		int firingWithinBlock = 0;

		if (CalibrationReportedNumLasers == 16)
		{
			assert(hdl64offset == 0);
			if (laserId >= 16)
			{
				laserId -= 16;
				firingWithinBlock = 1;
			}
		}

		// Interpolate azimuth
		double timestampadjustment = 0.0;
		double blockdsr0 = 0.0;
		double nextblockdsr0 = 1.0;
		if (CalibrationReportedNumLasers == 32)
		{
			timestampadjustment = HDL32AdjustTimeStamp(firingBlock, dsr);
			nextblockdsr0 = HDL32AdjustTimeStamp(firingBlock + 1, 0);
			blockdsr0 = HDL32AdjustTimeStamp(firingBlock, 0);
		}
		else if (CalibrationReportedNumLasers == 16)
		{
			timestampadjustment = VLP16AdjustTimeStamp(firingBlock, laserId, firingWithinBlock);
			nextblockdsr0 = VLP16AdjustTimeStamp(firingBlock + 1, 0, 0);
			blockdsr0 = VLP16AdjustTimeStamp(firingBlock, 0, 0);
		}
		int azimuthadjustment = Round(azimuthDiff * ((timestampadjustment - blockdsr0) / (nextblockdsr0 - blockdsr0)));
		timestampadjustment = Round(timestampadjustment);

		if (firingData->laserReturns[dsr].distance != 0.0 && LaserSelection[laserId])
		{
			PushFiringData(laserId,
				rawLaserId,
				azimuth + azimuthadjustment,
				timestamp + timestampadjustment,
				rawtime + static_cast<unsigned int>(timestampadjustment),
				&(firingData->laserReturns[dsr]),
				&(laser_corrections_[dsr + hdl64offset]),
				dual);
		}
	}
}



//-----------------------------------------------------------------------------
void MyHdlGrabber::SplitFrame(bool force)
{
	if (SplitCounter > 0 && !force)
	{
		SplitCounter--;
		return;
	}

	for (size_t n = 0; n < HDL_MAX_NUM_LASERS; ++n)
	{
		LastPointId[n] = -1;
	}
	endFrame = 1;
	//this->CurrentDataset->SetVerts(this->NewVertexCells(this->CurrentDataset->GetNumberOfPoints()));
	//this->Datasets.push_back(this->CurrentDataset);
	//this->CurrentDataset = this->CreateData(0);
}


//-----------------------------------------------------------------------------
void MyHdlGrabber::ProcessHDLPacket(unsigned char *data, std::size_t bytesReceived)
{
	if (bytesReceived != 1206)
	{
		return;
	}

	HDLDataPacket* dataPacket = reinterpret_cast<HDLDataPacket *>(data);

	//vtkNew<vtkTransform> geotransform;
	const unsigned int rawtime = dataPacket->gpsTimestamp;
	const double timestamp = 0;// ComputeTimestamp(dataPacket->gpsTimestamp);
	//ComputeOrientation(timestamp, geotransform.GetPointer());

	// Update the transforms here and then call internal
	// transform
	//  this->SensorTransform->Update();
	//geotransform->Update();

	int firingBlock = Skip;
	Skip = 0;

	std::vector<int> diffs(HDL_FIRING_PER_PKT - 1);
	for (int i = 0; i < HDL_FIRING_PER_PKT - 1; ++i)
	{
		int localDiff = (36000 + dataPacket->firingData[i + 1].rotationalPosition -
			dataPacket->firingData[i].rotationalPosition) % 36000;
		diffs[i] = localDiff;
	}
	std::nth_element(diffs.begin(),
		diffs.begin() + HDL_FIRING_PER_PKT / 2,
		diffs.end());
	int azimuthDiff = diffs[HDL_FIRING_PER_PKT / 2];
	assert(azimuthDiff >= 0);

	for (; firingBlock < HDL_FIRING_PER_PKT; ++firingBlock)
	{
		HDLFiringData* firingData = &(dataPacket->firingData[firingBlock]);
		int hdl64offset = (firingData->blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;
		IsHDL64Data |= (hdl64offset > 0);

		if (firingData->rotationalPosition < LastAzimuth)
		{
			SplitFrame(false);
		}

		// Skip this firing every PointSkip
		if (PointsSkip == 0 || firingBlock % (PointsSkip + 1) == 0)
		{
			ProcessFiring(firingData,
				hdl64offset,
				firingBlock,
				azimuthDiff,
				timestamp,
				rawtime);
		}

		LastAzimuth = firingData->rotationalPosition;
	}
}

//-----------------------------------------------------------------------------
bool MyHdlGrabber::GetFrame(int frameNumber)
{
	endFrame = 0;
	Points.clear();

	UnloadData();
	if (!Reader)
	{

		return 0;
	}
	if (!CorrectionsInitialized)
	{

		return 0;
	}

	assert(FilePositions.size() == Skips.size());
	if (frameNumber < 0 || frameNumber > FilePositions.size())
	{
		return 0;
	}

	const unsigned char* data = 0;
	unsigned int dataLength = 0;
	double timeSinceStart = 0;


	Reader->SetFilePosition(&FilePositions[frameNumber]);
	Skip = Skips[frameNumber];

	while (Reader->NextPacket(data, dataLength, timeSinceStart))
	{
		ProcessHDLPacket(const_cast<unsigned char*>(data), dataLength);

		if (endFrame)
		{
			return 0;// this->Internal->Datasets.back();
		}
	}

	SplitFrame(false);
	return 1;

	//return this->Internal->Datasets.back();
}

//-----------------------------------------------------------------------------
void MyHdlGrabber::Close()
{
	delete Reader;
	Reader = 0;
}


void MyHdlGrabber::Open()
{
	Close();
	Reader = new vtkPacketFileReader;
	if (!Reader->Open(FileName))
	{

		Close();
	}
}

