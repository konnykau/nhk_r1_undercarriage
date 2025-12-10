#pragma once
#include <math.h>
#include "nhk_r1_undercarriage/fry_lib/vector.hpp"
#include "nhk_r1_undercarriage/fry_lib/math.hpp"
#include "robomas_package_2/msg/motor_cmd_array.hpp"
#include "robomas_package_2/msg/motor_cmd.hpp"
#include <cstdint>


constexpr float cos45 = FRY::sqrt(2)/2;
// constexpr float cos30 = FRY::sqrt(3)/2;
// constexpr float sin30 = 0.5;

//きれいなコードにしたかったぁぁぁ....

class motor{//モーターのクラス
    private:
    const FRY::vec2d direction;//モーターの向いている方向ベクトル
    float TARGET_velocity;//TARGET(m/s)
    rclcpp::Logger logger_;//printfデバッグに使う
    const float gear_ratio_ = 19.0;//todo ギア比
    const float wheel_diameter_ = 0.126;//todo ホイール直径(m)
    float TARGET_rpm;//TARGETをrpmに変換したやつ
    public:
    motor(float x,float y)
    :direction(FRY::vec2d(x,y)),TARGET_velocity(0),logger_(rclcpp::get_logger("motor"))
    {}//初期化
    void set_target(float power){
        this->TARGET_velocity = power;
    }//TARGETを代入
    float make_frame()
    {
        //rpmに変換して返す
        this->TARGET_rpm = (this->TARGET_velocity * 60)/(3.14159265358979 * this->wheel_diameter_)*this->gear_ratio_;
        return this->TARGET_rpm;
    }
    FRY::vec2d get_vec2d(){
        return this->direction;
    }


};

enum class motor_mode{disable,velocity,position};//モーターのmode


class undercarriage{//足回り全体のクラス
    private:
    FRY::vec2d direction;//進みたい方向
    
    motor motors[4];
    uint8_t id_[4] = {1,2,3,4};
    //四輪オムニ    
    motor_mode MODE;
    rclcpp::Logger logger_;
    public:
    
    undercarriage()
    :direction(FRY::vec2d(0,0)),motors{motor(-cos45,cos45),motor(-cos45,-cos45),motor(cos45,-cos45),motor(cos45,cos45)},logger_(rclcpp::get_logger("undercarriage"))
    {
        MODE = motor_mode::disable;
    }//初期化
    void set_motor_power(float theta_vel);//4タイヤがうまく回るようにする
    void set_direction(float x,float y);//行きたい方向を設定
    std::unique_ptr<robomas_package_2::msg::MotorCmdArray> make_robomas_Frame();//robomas_package_2に送るメッセージを作成
    void make_mode(motor_mode motor_state);//modeを設定
    void update(float x,float y,float turn_dir);//他の関数を全部融合させた
};

inline void undercarriage::set_direction(float x,float y){
    this->direction.x = x;
    this->direction.y = y;
    //多分m/sになる
}

inline std::unique_ptr<robomas_package_2::msg::MotorCmdArray> undercarriage::make_robomas_Frame(){
    robomas_package_2::msg::MotorCmdArray TARGET_FRAME;
    uint8_t i = 0;
    for(motor m : this->motors){
        robomas_package_2::msg::MotorCmd cmd;
        cmd.id = this->id_[i];
        cmd.mode = 1;
        if(this->MODE == motor_mode::velocity){
            cmd.value = m.make_frame();
        }
        else{// if(this->MODE == motor_mode::disable)
            cmd.value = 0;
        }
        RCLCPP_INFO(logger_,"motor id:%d, value:%f",cmd.id,cmd.value);
        TARGET_FRAME.cmds.push_back(cmd);
        i++;
    }
    RCLCPP_INFO(logger_,"-------------------");
    
    return std::make_unique<robomas_package_2::msg::MotorCmdArray>(TARGET_FRAME);
}



inline void undercarriage::set_motor_power(float theta_vel){
    //この前段階でtheta_velは半径などとかけてm/sに変換されている想定

    for(motor &m : motors){
        float power = (this->direction * m.get_vec2d());
        m.set_target(power + theta_vel);
    }
}

inline void undercarriage::make_mode(motor_mode motor_state){
    this->MODE = motor_state;
}

inline void undercarriage::update(float x,float y,float turn_dir)
{
    this->set_direction(x,y);
    this->set_motor_power(turn_dir);
}




