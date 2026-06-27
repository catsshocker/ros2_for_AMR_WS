#ifndef UPDOWN_SPEED_HPP
#define UPDOWN_SPEED_HPP

#include <cmath>

class UpDownSpeed {
public:
    UpDownSpeed(){}
    UpDownSpeed(double S, double Vm, double Acc): Vm_(Vm),Acc_(Acc){
        this->S_ = S;
        this->sign_ = (S >= 0) ? 1 : -1; // 確定方向
        S = std::abs(S); // 取絕對值，因為速度規劃只關心距離大小，方向由 sign 決定
        // 梯形/三角形速度規劃邏輯 (Trapezoidal / Triangular Profile)
        // 臨界距離：剛好加速到 Vm 就立刻減速所需的總距離
        double d_critical = (Vm * Vm) / Acc;

        if (S >= d_critical) {
            // 情況 A：距離夠長，可以達到最大速度 Vm（梯形速度曲線）
            double t_acc = Vm / Acc;                               // 加速所需時間
            double t_constant = (S - d_critical) / Vm;       // 等速所需時間

            this->t1 = t_acc;
            this->t2 = t_acc + t_constant;
            this->t3 = this->t2 + t_acc;
            this->v_max_calc = Vm;
        }
        else {
            // 情況 B：距離太短，還沒到 Vm 就得開始減速（三角形速度曲線）
            // 此時實際能達到的最高速度 v_peak = sqrt(S * Acc)
            double v_peak = std::sqrt(S * Acc);
            double t_acc = v_peak / Acc;
            
            this->t1 = t_acc;
            this->t2 = t_acc; // 沒有等速階段
            this->t3 = t_acc + t_acc;
            this->v_max_calc = v_peak;
        }
    }

    double update(double dt, int i){ 
        double t = i*dt;
        double v_out = 0.0;
        if(t<=this->t1)         {v_out = this->Acc_ * t ;}
        else if(t <= this->t2)  {v_out = this->v_max_calc;}
        else if(t <= this->t3)  {v_out = this->Acc_ * (this->t3 - t);}
        else{
            this->PR_ = 1.0;
            this->is_finished = true;
            return 0;
        }
        this->Area_ += std::abs(v_out) * dt;
        this->PR_ = (this->Area_ / this->S_);
        return v_out * this->sign_;
    }

    bool finished(){
        return this->is_finished;
    }

    double getT1() const { return t1; }
    double getT2() const { return t2; }
    double getT3() const { return t3; }

    double getPR() const { return PR_;} // get % 
    double getArea() const { return Area_; }

private:
    double S_;
    double Vm_;      // 最大速度
    double Acc_;     // 加速度
    double t1,t2,t3;   // 目標速度
    double v_max_calc; // 實際能達到的最大速度
    bool is_finished = false; // 是否完成速度規劃
    int sign_; // 方向標誌，1 表示正方向，-1 表示負方向
    double Area_ ;
    double PR_ ; // %

};


#endif