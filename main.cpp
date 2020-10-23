#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "find_closest.h"
#include "residual.h"
#include <fstream>

using namespace std;

int main() {
    Eigen::Quaterniond q(0.999803474666, -0.000201179970759,
                         -0.000247586521001, -0.0198219644252);
    Eigen::Vector3d position1(0.449226915836, 0.0111121777445, 0);
    //Eigen::Vector3d position2(0.3, 0.1, 0);
    Eigen::Vector3d eulerAngle=q.matrix().eulerAngles(2,1,0);
    vector<Eigen::Vector3d> observation;
    vector<Eigen::Vector3d> map_point;
    Eigen::Vector3d observation_1(-2.2826859951, 12.2115087509, 0.111243285239);
    Eigen::Vector3d observation_2(0.729999005795, 12.1273288727, 0.0745350643992);
    Eigen::Vector3d observation_3(0.788380503654, 5.33485031128, 0.053799752146);
    Eigen::Vector3d observation_4(3.54080843925, 12.0889282227, 0.0512603223324);
    Eigen::Vector3d observation_5(7.76460790634, 18.8468933105, 0.111923471093);
    Eigen::Vector3d observation_6(3.1900434494, 5.71377182007, 0.0502007715404);
    Eigen::Vector3d observation_7(9.45205879211, 11.8226194382, 0.118333414197);
    Eigen::Vector3d observation_8(12.3625793457, 11.8424339294, 0.0686551183462);
    Eigen::Vector3d observation_9(7.09109687805, 5.02175760269, 0.0621468238533);
    Eigen::Vector3d observation_10(7.33838224411, -1.53285241127, 0.0641292408109);
    Eigen::Vector3d observation_11(-0.39133232832, -11.9953975677, 0.0689698979259);
    Eigen::Vector3d observation_12(-3.34272050858, -11.6464042664, 0.0767287015915);
    Eigen::Vector3d observation_13(-3.4076230526, -11.8592691422, 0.0511717535555);
    Eigen::Vector3d observation_14(-4.39138460159, -12.0333909988, 0.0741704404354);
    Eigen::Vector3d observation_15(-11.3335208893, -18.1205177307, 0.0689721032977);
    Eigen::Vector3d observation_16(-12.1436681747, 6.03326225281, 0.109609730542);
    Eigen::Vector3d observation_17(-5.16634559631, 5.4313249588, 0.0884011611342);
    Eigen::Vector3d observation_18(-8.19388580322, 12.4914093018, 0.0846289619803);
    observation.push_back(observation_1);
    observation.push_back(observation_2);
    observation.push_back(observation_3);
    observation.push_back(observation_4);
    observation.push_back(observation_5);
    observation.push_back(observation_6);
    observation.push_back(observation_7);
    observation.push_back(observation_8);
    observation.push_back(observation_9);
    observation.push_back(observation_10);
    observation.push_back(observation_11);
    observation.push_back(observation_12);
    observation.push_back(observation_13);
    observation.push_back(observation_14);
    observation.push_back(observation_15);
    observation.push_back(observation_16);
    observation.push_back(observation_17);
    observation.push_back(observation_18);


    Eigen::Vector3d map_point_1(0-4.4175362587, 5.67111968994, 0.0861253365874);
    Eigen::Vector3d map_point_2(-7.1919503212, 12.7840385437, 0.088314011693);
    Eigen::Vector3d map_point_3(1.63929033279, 12.0983505249, 0.0736672431231);
    Eigen::Vector3d map_point_4(3.32956457138, 19.1917686462, 0.0655490383506);
    Eigen::Vector3d map_point_5(1.45116794109, 5.30915117264, 0.0679405704141);
    Eigen::Vector3d map_point_6(4.48962259293, 11.9310693741, 0.0597683340311);
    Eigen::Vector3d map_point_7(8.85734272003, 18.5377731323, 0.108130931854);
    Eigen::Vector3d map_point_8(11.8424882889, 18.0297031403, 0.100742869079);
    Eigen::Vector3d map_point_9(3.89341282845, 5.60199594498, 0.0784432739019);
    Eigen::Vector3d map_point_10(10.3568029404, 11.3838739395, 0.0994828045368);
    Eigen::Vector3d map_point_11(13.2534837723, 11.4107990265, 0.0682683363557);
    Eigen::Vector3d map_point_12(7.72590875626, 4.74846935272, 0.0688899978995);
    Eigen::Vector3d map_point_13(16.0163974762, -2.11401820183, 0.0983867645264);
    Eigen::Vector3d map_point_14(10.6637639999, -2.04198336601, 0.0854744017124);
    Eigen::Vector3d map_point_15(7.71824407578, -1.83371376991, 0.0822677761316);
    Eigen::Vector3d map_point_16(4.7843503952, -1.88496136665, 0.0558005608618);
    Eigen::Vector3d map_point_17(4.77871751785, -12.9306411743, 0.0534234493971);
    Eigen::Vector3d map_point_18(-3.4142203331, -11.6952152252, 0.0542656369507);
    Eigen::Vector3d map_point_19(-3.41789960861, -11.497756958, 0.0611909925938);
    Eigen::Vector3d map_point_20(-11.2032527924, 11.6839132309, 0.0744699239731);
    Eigen::Vector3d map_point_21(-11.4929075241, 6.47847127914, 0.108309216797);
    Eigen::Vector3d map_point_22(-11.3600406647, 6.76245641708, 0.0742002427578);
    Eigen::Vector3d map_point_23(-11.2093019485, 6.83765792847, 0.166255772114);
    Eigen::Vector3d map_point_24(-10.85546875, 9.08504676819, 0.0693461149931);
    Eigen::Vector3d map_point_25(-1.28285884857, 12.3902730942, 0.067250661552);
    map_point.push_back(map_point_1);
    map_point.push_back(map_point_2);
    map_point.push_back(map_point_3);
    map_point.push_back(map_point_4);
    map_point.push_back(map_point_5);
    map_point.push_back(map_point_6);
    map_point.push_back(map_point_7);
    map_point.push_back(map_point_8);
    map_point.push_back(map_point_9);
    map_point.push_back(map_point_10);
    map_point.push_back(map_point_11);
    map_point.push_back(map_point_12);
    map_point.push_back(map_point_13);
    map_point.push_back(map_point_14);
    map_point.push_back(map_point_15);
    map_point.push_back(map_point_16);
    map_point.push_back(map_point_17);
    map_point.push_back(map_point_18);
    map_point.push_back(map_point_19);
    map_point.push_back(map_point_20);
    map_point.push_back(map_point_21);
    map_point.push_back(map_point_22);
    map_point.push_back(map_point_23);
    map_point.push_back(map_point_24);
    map_point.push_back(map_point_25);
    //Eigen::Vector3d w_point_1 = point_trans(observation_1, q, position1);
    //Eigen::Vector3d cor_point_1 = find_closest(w_point_1, map_point);
    //vector<Eigen::Vector3d>::iterator iter = map_point.begin();
    //cout << (*iter)[0] << endl;
    //Eigen::Vector3d match;
    //match = *iter;
    //cout << match << endl;
    //cout << w_point_1 << endl;
    //cout << cor_point_1 << endl;
    //cout << w_point_1 << endl;

    // hange the angle
    std::ofstream fout("/home/xcy/cpp_txt/angel_data_0.01");
    int count = 0, min = 100;
    for(double i = 3.1412; i >= -3.14; i -= 0.01)
    {
        Eigen::Vector3d eulerAngle(3.10195,-3.14109,i);
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ()));

        Eigen::Quaterniond quaternion;
        quaternion=yawAngle*pitchAngle*rollAngle;
        double res = residual(position1, observation, quaternion, map_point);
        ++count;
        min = res < min ? res : min;
        fout << res << "\t";
        if(count % 10 == 0)
            fout << endl;
    }

//    double res = residual(position1, observation, q, map_point);
//    cout << res << endl;
//    //double res2 = residual(position2, observation, q, map_point);
//    //cout << res2 << endl;
//
//    double boundary = 20.00;
//    double step = 0.05;
//
//    int count = 0;
//    for(double i = 0; i <= 0.30; i += 0.05)
//    {
//        for(double j = -9.5; j <= -8.0; j += 0.05)
//        {
//            Eigen::Vector3d position(i , j, 0);
//            double res = residual(position, observation, q, map_point);
//            ++count;
//            //min = res < min ? res : min;
//            cerr << res << "\t";
//            if(count % int(2 * boundary / step + 1) == 0)
//                cout << endl;
//        }
//    }

//    std::ofstream fout("/home/xcy/cpp_txt/data_20_0.05");
//    double min = 100;
//    for(double i = -boundary; i <= boundary + step; i += step){
//        for(double j = -boundary; j <= boundary + step; j += step)
//        {
//            Eigen::Vector3d position(i , j, 0);
//            double res = residual(position, observation, q, map_point);
//            ++count;
//            min = res < min ? res : min;
//            fout << res << "\t";
//            if(count % int(2 * boundary / step + 1) == 0)
//            //if(count % 401 == 0)
//                fout << endl;
//        }
//    }
//    fout.close();
//    cerr << min << endl;

    cout << eulerAngle << endl;

    return 0;
}
