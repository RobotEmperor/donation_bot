/*
 * motor.cpp
 *
 *      Author: robotemperor
 */
#include <robot1/motor.h> // 변수와 클래스 선언 헤더 추가

DcMotorForRaspberryPi::DcMotorForRaspberryPi() // 클래스 기본 생성자 선언
{
}
DcMotorForRaspberryPi::DcMotorForRaspberryPi(int encoder_pulse_per_rotation, int control_freqency, int channel):
    encoder_pulse_per_rotation_(encoder_pulse_per_rotation),
    control_freqency_(control_freqency),
    channel_(channel) // 클래스 생성시 초기화 변수 1회전당 엔코더 펄스수, 제어주기, 엔코더를 몇채배 사용 할 것 인지를 선언 해준다.
{
  encoder_pulse1 = 0; // 엔코더 속도 측정 변수1 선언 (1채널)
  encoder_pulse2 = 0; // 엔코더 속도 측정 변수2 선언 (2채널)

  encoder_pulse_position1 = 0; // 엔코더 속도 측정 변수1 선언 (1채널)
  encoder_pulse_position2 = 0; // 엔코더 속도 측정 변수2 선언 (2채널)

  acceleration_value = 40; // rpm 4.18 rad/s^2 속도 프로파일 가속도 지정 (유저에 따라 변경 가능)

  p_gain_position_ = 0.01; // 위치 p제어기의 게인 값
  p_gain_speed_ = 0.5; // 속도 p제어기의 게인값

  pwm_value_motor = 0; // 모터의 최종 출력으로 나오는 pwm 값 변수

  direction = 0;// 모터의 방향 변수
  check_position_control= 0; //모터 위치 제어 완료/미완료 체크 변수
  onoff = 0; // 모터 on off 변수

  speed_motor = 0; // 제어하고자 하는 속도 변수
  angle_motor = 0; // 제어하고자 하는 위치 변수
  result_rpm = 0; // 엔코더를 통해 출력되는 속도 변수

  speed_static_encoder_pulse_ = 0; // 위치를 측정하기 위한 엔코더 변수
  speed_error_ = 0; // 실제 측정된 상대적인 위치값과 제어하고자 하는 위치값의 에러 변수
  speed_control_ = 0; // 모터 속도 컨트롤 값 변수

  position_static_encoder_pulse_ = 0; // 위치를 측정하기 위한 엔코더 변수
  position_error_ = 0; // 실제 측정된 상대적인 위치값과 제어하고자 하는 위치 값의 에러 변수
  position_control_ = 0; // 모터 위치 컨트롤 값 변수

  position_max_rpm = 0; // 위치 제어시 최대 출력 제한
  check_position = true; // 위치 제어 완료/ 미완료 체크 변수
}
DcMotorForRaspberryPi::~DcMotorForRaspberryPi()
{
} // DC 모터 클래스 소멸자 변수
void DcMotorForRaspberryPi::speed_controller(int desired_speed)// 모터 속도 제어기 함수 입력은 원하는 스피드 / 출력은 pwm 값
{
  speed_static_encoder_pulse_ = (encoder_pulse1+ encoder_pulse2)*0.2 + speed_static_encoder_pulse_*0.8; // digital low pass filter  // basic 2 ch
  encoder_pulse1 = 0; // 제어 주기 마다 들어오는 펄스 수를 알기 위해 엔코더 펄스수를 초기화 해줘야함
  encoder_pulse2 = 0; // ""
  result_rpm =  (((speed_static_encoder_pulse_)*60*control_freqency_)/(encoder_pulse_per_rotation_*channel_)); // 엔코더 펄스 수를 기반으로 속도 측정 방법 (M 방식 채택) 자세한 설명은 파워포인트 참고

  speed_error_ = desired_speed  -  result_rpm ; //원하는 속도값과 실제 출력된 속도값의 에러값 측정
  speed_control_ = ( p_gain_speed_ * speed_error_); // 에러값에 게인 값을 적용

  pwm_value_motor = (pwm_value_motor + speed_control_); // pwm 값은 - 가 없고 항상 양수이기 때문에 에러값을 기존의 pwm 값에 더하면서 값을 출력한다.

  if (pwm_value_motor > 512) // pwm 값 제한
  {
    pwm_value_motor = 512;
  }

  if (pwm_value_motor < 0) // - 값이 계산 된다면, 0 으로 출력
  {
    pwm_value_motor = 0;
  }

}
double DcMotorForRaspberryPi::position_controller(int desired_angle, int max_rpm) // 위치 제어기 함수  참고용
{
  position_static_encoder_pulse_ = (encoder_pulse_position1+ encoder_pulse_position2);

  if(((desired_angle*encoder_pulse_per_rotation_*channel_)/360) <= position_static_encoder_pulse_)
  {
    check_position = true;
    return 0;
  }
  else
  {
    position_error_ = ((desired_angle*encoder_pulse_per_rotation_*channel_)*360) - position_static_encoder_pulse_;
    position_control_ = p_gain_position_ * position_error_;

    if(position_control_ >  max_rpm)
    {
      position_control_ = max_rpm;
    }
    if(position_control_ < 0)
    {
      position_control_ = 0;
    }
    return position_control_;
  }

}






















