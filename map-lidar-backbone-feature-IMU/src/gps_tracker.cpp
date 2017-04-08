#include "imu_tracker/gps_tracker.h"
#include <math.h>
#include <stdlib.h>

GPSTracker::GPSTracker(const double latitude, const double longitude, const double alpha, const double spd_cutoff)
  :  lat_(latitude),
     long_(longitude),
     raw_lat_(latitude),
     raw_long_(longitude),
     alpha_(alpha),
     spd_ct_(spd_cutoff) {}

void GPSTracker::AddLatLong(const double latitude, const double longitude,
	const double spd) {
	if (raw_lat_ == latitude || raw_long_ == longitude || abs(spd) < spd_ct_) {
		return;
	} else {
		raw_lat_ = latitude;
		raw_long_ = longitude;
		double temp_lat_ = lat_ + alpha_ * (raw_lat_ - lat_);
		double temp_long_ = long_ + alpha_ * (raw_long_ - long_);
		double dy = temp_lat_ - lat_;
		double dx = cos(M_PI / 180 * lat_) * (temp_long_ - long_);
		heading_ = atan2(dy, dx);
		lat_ = temp_lat_;
		long_ = temp_long_;
	}
}

double GPSTracker::getHeading() {
	return heading_;
}