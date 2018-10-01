/*
 * motor_algorithm.cpp
 *
 *      Author: robotemperor
 */
#include <robot1/motor_algorithm.h>// 변수와 함수들이 선언 되어 있는 헤더 추가

////////////////////////////////////////////////////////////////////////////// 모터 속도의 선형 가속을 위한 함수 생성 클래스
TrajectoryGenerator::TrajectoryGenerator() //
{
  pre_desired_value = 0; // 바로 이전 제어 루프에서 생성된 값 저장 변수
  current_desired_value = 0; // 현재 제어 루프에서 생성된 값 저장 변수
  out_value = 0; // 최종 출력 값
  time_count = 0; // 시간 카운트 변수
  tra_done_check = 0; // 함수가 원하는 변수 값에 도달 했는지 안했는지 판단 하는 변수

}
TrajectoryGenerator::~TrajectoryGenerator() //소멸자
{

}
double TrajectoryGenerator::linear_function(double desired_value, double acceleration)// y= ax +b 선형 함수 사용 / 원하는 값 / 정상상태까지 도달하기 위한 가속도 지정
{
  static double time = 0; // 시간 초기화 (함수를 불러올때 처음에만 초기화, --> static)

  if(current_desired_value != desired_value) // 예외사항 -> 트레젝토리가 끝나지 않았는데, 새로운 명령이 들어왔을때 처리.
  {
    time_count = 0; // 다시 함수 값 생성 시작
    tra_done_check = false; // 함수 값이 다시 생성 시작이니, 미완료로 판단 변수 저장 (완료 true / 미완료 false)
    pre_desired_value = out_value; // 이전 생성된 명령값은 당연히 새로운 명령이 들어오기 전의 마지막 출력값
  }

  time = fabs((pre_desired_value - desired_value) / acceleration); // 사용자가 정한 가속도 값에 따른 시간 출력

  current_desired_value = desired_value; // 현재 도달하고자 하는 값은 사용자가 입력한 값 (함수 입력인자)

  //time_count = time_count + 0.01; // 10ms 마다 새로운 값을 출력

  if(time_count >= time) // 사용자가 정한 가속도 값에 따른 시간 보다 시간 카운트가 높으면 원하는 값에 도달이 완료 됨
  {
    tra_done_check = true; // 정상상태 도달 완료.
    pre_desired_value = desired_value; // 이전 원하던 값은 사용자가 원하는 값 (도달 했으니까)
    return pre_desired_value; // 새로운 명령이 들어 올때까지 이전 명령값을 출력
  }

  if(pre_desired_value != desired_value && tra_done_check == false) // 트레젝토리가 함수 사용자가 원하는 값에 도달하지 않았을때,
  {
    time_count = time_count + 0.01; // 10ms 마다 시간 카운트

    if(pre_desired_value > desired_value) // 하강 트레젝토리 y = -at + b
    {
      out_value = -(acceleration)*time_count + pre_desired_value; //이전 값에서 시작 (절편)
      return out_value; //함수의 최종 출력 값
    }
    if(pre_desired_value < desired_value)// 상승 트레젝토리 y = at + b
    {
      out_value = (acceleration)*time_count + pre_desired_value; //이전 값에서 시작 (절편)
      return out_value; //함수의 최종 출력 값
    }
  }
  else
  {
    return pre_desired_value; // 함수가 사용자가 원하는 값에 도달 했다면, 이전 원하는 값으로 출력
  }

}
//////////////////////////////////////////////////////////////////////////////
void initialize() //모터 알고리즘 변수 초기화
{
  //reference_angle = 0;
  //reference_distance = 0;

  motor1 = new DcMotorForRaspberryPi(399,100,2);// 모터 라이브러리 사용 간접 참조로 재선언/ 엔코더 펄스수 1회전당 399 / 제어주기 100hz / 2체배 사용
  motor2 = new DcMotorForRaspberryPi(399,100,2);

  tra_motor1 = new TrajectoryGenerator;// 모터 1 함수 생성 클래스 간접 참조로 재선언
  tra_motor2 = new TrajectoryGenerator; // 모터 2 함수 생성 클래스 간접 참조로 재선언

  current_desired_speed_motor1 = 0; // 현재 원하는 모터1 스피드 값
  current_desired_speed_motor2 = 0; // 현재 원하는 모터2 스피드 값

  motor1->check_position = true;  //위치 제어 완료 변수
  motor2->check_position = true;
}
void motor1_encoder_1(void) // 모터 엔코더 인터럽트 함수
{
  //인터럽트 함수는 엔코더 펄스가 들어올때마다 실행되며 그 때마다 카운트 모터1 1채널
  motor1->encoder_pulse1 ++;
  motor1->encoder_pulse_position1 ++;
}
void motor1_encoder_2(void)
{
  //인터럽트 함수는 엔코더 펄스가 들어올때마다 실행되며 그 때마다 카운트 모터1 2 채널
  motor1->encoder_pulse2++;
  motor1->encoder_pulse_position2 ++;
}

void motor2_encoder_1(void)
{
  //인터럽트 함수는 엔코더 펄스가 들어올때마다 실행되며 그 때마다 카운트 모터2 1채널
  motor2->encoder_pulse1 ++;
  motor2->encoder_pulse_position1 ++;
}
void motor2_encoder_2(void)
{
  //인터럽트 함수는 엔코더 펄스가 들어올때마다 실행되며 그 때마다 카운트 모터2 2채널
  motor2->encoder_pulse2 ++;
  motor2->encoder_pulse_position2 ++;
}
//test
void motor_test_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)// 모터 제어 테스트 / 원하는 값 입력 받는 메세지 콜백 함수
{
  motor1->speed_motor = msg->data[0]; // 배열 0번은 모터1 스피드 입력
  motor2->speed_motor = msg->data[1]; // 배열 1번은 모터 2 스피드 입력

  motor1->check_position = false; // 위치 제어 완료 변수
  motor2->check_position = false;
}
void algorithm(double angle, double distance) // 위치 제어 알고리즘 참고용 // 각도와 거리가 주어지면(입력) 주어진 각도만큼 돌고 그 다음 바로 직선거리로 주어진 거리만큼 진행하는 알고리즘
{
  static int motion_sequence = 1;
  if(motor1->check_position == true && motor2->check_position == true && motion_sequence == 1)
  {
    motion_sequence ++;
    motor1->encoder_pulse_position1 = 0;
    motor1->encoder_pulse_position2 = 0;
    motor2->encoder_pulse_position1 = 0;
    motor2->encoder_pulse_position2 = 0;

    motor1->check_position = false;
    motor2->check_position = false;
    printf("Motion change \n");
  }
  else if(motor1->check_position == true && motor2->check_position == true && motion_sequence == 2)
  {
    motion_sequence ++;
    angle_control_done_msg.data = "done";


    motor1->angle_motor = 0;
    motor2->angle_motor = 0;

    printf("Motion done! \n");
  }
  else if(motor1->check_position == true && motor2->check_position == true && motion_sequence == 3)
  {
    printf("waiting! \n");
  }
  else if(motor1->check_position == false && motor2->check_position == false && motion_sequence == 3)
  {
    motion_sequence = 1;
    printf("Motion init! \n");
  }
  else
  {
    printf("running! \n");
  }
  switch(motion_sequence)
  {
  case 1 :
  {
    motor1->position_max_rpm = 20;
    motor2->position_max_rpm = 20;
    motor1->angle_motor = (int) fabs(angle*2);////
    motor2->angle_motor = motor1->angle_motor;

    if(angle < 0)
    {
      motor1->direction = false;
      motor2->direction = false;
    }
    else
    {
      motor1->direction = true;
      motor2->direction = true;
    }
    break;
  }
  case 2 :
  {
    motor1->position_max_rpm = 50;
    motor2->position_max_rpm = 50;
    motor1->angle_motor = (int) ((fabs(distance)/0.035)*180)/M_PI; ///
    motor2->angle_motor = motor1->angle_motor;
    if(distance > 0)
    {
      motor1->direction = true;
      motor2->direction = true;
    }
    if(distance < 0)
    {
      motor1->direction = false;
      motor2->direction = false;
    }
    break;
  }
  default :
    break;
  }
}
/////////////////////////////////////////////////////////////////////////////////////
void motor_control(int id, int motor_line1, int mode, bool direction, int desired_speed_rpm, int angle, bool on_off) // 모터 제어 알고리즘 함수 // 모터의 아이디, 모터 방향 입력 핀, 제어모드, 모터 방향, 원하는 스피드, 원하는 각도, 모터 온오프
{
  if(on_off == true) // 모터 온 일때,
  {
    if(direction == true)//CW
    {
      digitalWrite(motor_line1,HIGH); // wiringPi 라이브러리를 활용하여 특정핀의 출력을 HIGH로 설정함
    }
    else if (direction == false)//CCW
    {
      digitalWrite(motor_line1,LOW); // wiringPi 라이브러리를 활용하여 특정핀의 출력을 LOW로 설정함
    }
    // 수정 되어 질 수 있음

    switch (id) // 하나의 모터 컨트롤 함수로 두개의 모터를 제어하기위한 스위치 함수
    {
    case 1 : // 1번 모터
      if(mode == 1)// 위치 제어 모드 일때,
      {
        desired_speed_rpm = motor1->position_controller(angle, motor1->position_max_rpm);// 위치 컨트롤러
      }
      // 속도 제어 모드일때는 위의 코드가 실행 되지않고 바로 리니어 함수로 출력값 생성
      current_desired_speed_motor1 = tra_motor1->linear_function(desired_speed_rpm, motor1->acceleration_value); // 위치 제어 컨트롤러에서 나온 최종 출력(원하는 속도)를 리니어 함수로 원하는 가속도에 의해 출력
      desired_speed_rpm = current_desired_speed_motor1;// 원하는 속도 값 재설정
     
      motor1->speed_controller(desired_speed_rpm); // 원하는 속도 값에 따른 속도 컨트롤러
      break;
    case 2 :// 모터 2
      if(mode == 1)
      {
        desired_speed_rpm = motor2->position_controller(angle, motor2->position_max_rpm);
      }
      current_desired_speed_motor2 = tra_motor2->linear_function(desired_speed_rpm, motor2->acceleration_value);
      desired_speed_rpm = current_desired_speed_motor2;

      motor2->speed_controller(desired_speed_rpm);
      break;
    default : // 그외 값을 들어왔을 때, break;
      break;
    }
  }

  if(on_off == false) // 모터 오프 상태일 때,
  {
    switch (id) // 하나의 모터 컨트롤 함수로 두개의 모터를 제어하기위한 스위치 함수
        {
        case 1 : // 1번 모터
          pwmWrite(motor1_PWM, 0); // WiringPi 라이브러리 활용하여 pwm 핀에 pwm 출력값을 0 으로 설정 모터1
          break;
        case 2 :// 모터 2
          pwmWrite(motor2_PWM, 0); // WiringPi 라이브러리 활용하여 pwm 핀에 pwm 출력값을 0 으로 설정 모터2
          break;
        default : // 그외 값을 들어왔을 때, break;
          break;
        }
  }
}

void controlFunction(const ros::TimerEvent&) // 10ms의 제어주기를 갖는 제어 함수 ROS timer를 이용
{
  //두개의 모터 ON
  motor1->onoff = 1;
  motor2->onoff = 1;

  //algorithm(reference_angle, reference_distance);

  motor_control(1, motor1_IN1, 0,  motor1->direction, motor1->speed_motor, motor1->angle_motor, motor1->onoff);// 모터1 컨트롤러
  motor_control(2, motor2_IN1, 0,  motor2->direction, motor2->speed_motor, motor2->angle_motor, motor2->onoff);// 모터2 컨트롤러

  //최종 출력된 모터 PWM 값을 WiringPi 라이브러리를 활용하여 pwm 값을 설정
  pwmWrite(motor1_PWM, (int) motor1->pwm_value_motor);// 모터 1
  pwmWrite(motor2_PWM, (int) motor2->pwm_value_motor);//모터 2

}
int main (int argc, char **argv)// 메인 함수
{
  wiringPiSetupGpio();// wiring Pi 라이브러리를 사용 하기위해 초기화

  initialize(); // 변수 초기화

  ros::init(argc, argv, "motor_node"); // ROS 노드 초기화
  ros::NodeHandle nh; // ROS 노드 핸들러 선언

  ros::Timer timer_control = nh.createTimer(ros::Duration(0.01), controlFunction); // 10ms 타이머 생성 // 10ms 마다 controlFunction을 실행해라.

  motor_theta_dist_sub   = nh.subscribe("motor_theta_dist", 1, motor_test_callback); // test subscriber

  // 모터 1 2의 현재 속도 값과 원하는 속도값을 publish 하기위한 선언 부
  result_rpm1_pub = nh.advertise<std_msgs::Float64>("result_rpm1",10);
  result_rpm2_pub = nh.advertise<std_msgs::Float64>("result_rpm2",10);
  desired_rpm1_pub = nh.advertise<std_msgs::Float64>("desired_rpm1",10);
  desired_rpm2_pub = nh.advertise<std_msgs::Float64>("desired_rpm2",10);

  // Wiring Pi GPIO 핀을 사용하기 위해 핀모드를 설정
  pinMode(motor1_IN1, OUTPUT); // CW CCW 출력
  pinMode(motor1_FG1, INPUT); //모터1 엔코더 1 입력 (라즈베리파이로)
  pinMode(motor1_FG2, INPUT); //모터1 엔코더 2 입력 (라즈베리파이로)

  pinMode(motor2_IN1, OUTPUT); // CW CCW 출력
  pinMode(motor2_FG1, INPUT);  //모터1 엔코더 1 입력 (라즈베리파이로)
  pinMode(motor2_FG2, INPUT);  //모터2 엔코더 2 입력 (라즈베리파이로)

  //엔코더 펄스가 들어올때마다 엔코드 펄수를 카운팅 하기위한 GPIO 인터럽트 함수 선언
  wiringPiISR(motor1_FG1, INT_EDGE_RISING, &motor1_encoder_1); // motor1_FG1의 핀으로, Rising Edge 로 펄스가 들어 올때, motor1_encoder1 함수를 실행한다.
  wiringPiISR(motor1_FG2, INT_EDGE_RISING, &motor1_encoder_2); // motor1_FG2의 핀으로, Rising Edge 로 펄스가 들어 올때, motor1_encoder2 함수를 실행한다.
  wiringPiISR(motor2_FG1, INT_EDGE_RISING, &motor2_encoder_1); //
  wiringPiISR(motor2_FG2, INT_EDGE_RISING, &motor2_encoder_2); //

  pinMode(motor1_PWM, PWM_OUTPUT); // 하드웨어 pwm 을 사용하기 위한 핀 모드 설정
  pinMode(motor2_PWM, PWM_OUTPUT); //
  pwmSetMode (PWM_MODE_MS); // pwm 모드 설정
  pwmSetRange(512); // 범위는 0-512
  pwmSetClock(2);   // pwm 클럭 조정

  // pwm frequency = 19.2M / pwm set range / pwm set clock

  digitalWrite(motor1_IN1,HIGH); // motor1_IN1의 핀을 HIGH로 설정한다
  digitalWrite(motor2_IN1,LOW);  // motor2_IN1의 핀을 LOW로 설정한다


  pwmWrite(motor1_PWM, 0); // 모터 1 pwm 출력 값 0 으로 초기화
  pwmWrite(motor2_PWM, 0); // 모터 2 pwm 출력 값 0 으로 초기화

  while(ros::ok()) // ROS 노트 상태가 true일때 반복
  {
    // 주석 처리된 코드를 주석을 해제하면 원하는 속도값과 실제 출력값을 publish 함
    //desired_rpm1_msg.data = current_desried_speed_motor1;
    //desired_rpm2_msg.data = current_desried_speed_motor2;
    //result_rpm1_msg.data = motor1->result_rpm;
    //result_rpm2_msg.data = motor2->result_rpm;
    usleep(100); // cpu의 사용률을 줄이기 위한 슬립
    //result_rpm1_pub.publish(result_rpm1_msg);
    //result_rpm2_pub.publish(result_rpm2_msg);
    //desired_rpm1_pub.publish(desired_rpm1_msg);
    //desired_rpm2_pub.publish(desired_rpm2_msg);

    ros::spinOnce();// ROS 노드 업데이트
  }

  // 사용된 메모리 클래스 삭제
  delete motor1;
  delete motor2;
  delete tra_motor1;
  delete tra_motor2;

  // 모터 출력값을 0으로 설정하여 off
  pwmWrite(motor1_PWM, 0);
  pwmWrite(motor2_PWM, 0);

  return 0;
}


