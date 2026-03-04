#ifndef ELI_CS_ROBOT_CALIBRATION_CALIBRATION_LIBRARY_HPP_
#define ELI_CS_ROBOT_CALIBRATION_CALIBRATION_LIBRARY_HPP_

namespace eli_cs_robot_calibration
{

class CalibrationLibrary
{
public:
  CalibrationLibrary();
  virtual ~CalibrationLibrary();
  
  void initialize();
  void performCalibration();
};

}  // namespace eli_cs_robot_calibration

#endif  // ELI_CS_ROBOT_CALIBRATION_CALIBRATION_LIBRARY_HPP_
