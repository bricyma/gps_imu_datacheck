#ifndef GPS_TRACKER_H_
#define GPS_TRACKER_H_

class GPSTracker {
  // Low pass filter for gps trajectory
  public:
    GPSTracker(const double latitude, const double longitude, 
      const double alpha, const double spd_cutoff);
    void AddLatLong(const double latitude, const double longitude, const double spd);
    double getHeading();
  private:
    // Current filtered location
    double lat_, long_;
    // Current unfiltered location
    double raw_lat_, raw_long_;
    // Current heading angle using filtered location
    double heading_;
    // Speed value below which heading is fixed
    const double spd_ct_;
    // Low pass filter parameter, sample differential when alpha equals 1
    const double alpha_;
};

#endif // GPS_TRACKER_H_