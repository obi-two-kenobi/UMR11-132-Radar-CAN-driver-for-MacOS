
#include "UMR11132.hpp"

int main() {
    cv::viz::Viz3d window("Radar viz");
    cv::namedWindow("Control", cv::WINDOW_NORMAL);

    Sensor::toolBox tb;
    Sensor::Offline_UMR11 radar(tb,"19AprilFriday.csv");
// or
//  Lidar::CAN_UMR11 radar(tb);
    
    int i;
    
    while (true)
    {
        auto data = radar.getFrame();
        i=1;
        std::cout << "Cycle: " << radar.getCycleCount() << " | #Targets: " << radar.getnumberOfTargets()<< " | Cycle Duration: " << radar.getCycleDuration() << "s | Time Stamp: " << radar.getTimeStamp() << "s\n" ;
        for (auto& point : data)
        {
           
            std::cout << "ID : " << point.ObjID <<"\n"
                      << "|\tR  : " <<  point.R <<" m\n"
                      << "|\tRR : " << point.RR <<" m/s\n"
                      << "|\tAZT: " <<  point.Azimuth <<" deg\n"
                      << "|\tELV: " << point.Elevation <<" deg\n"
                      << std::endl;
            
            int X3d = point.R * cos(toRad(point.Azimuth)) * cos(toRad(point.Elevation));
            int Y3d = point.R * sin(toRad(point.Azimuth)) * cos(toRad(point.Elevation));
            int Z3d = point.R * sin(toRad(point.Elevation));
            
            window.showWidget(std::to_string(i), cv::viz::WSphere(cv::Point3d(X3d,Y3d,Z3d), 0.2));
            window.showWidget("strng"+std::to_string(i++),cv::viz::WText3D(std::to_string((int)round(point.R)), cv::Point3d(X3d,Y3d,Z3d+0.3),0.2));
        }
        
        window.showWidget("gimble", cv::viz::WCoordinateSystem(1));
        window.showWidget("grid", cv::viz::WGrid());
        window.spinOnce(1, true);
        cv::waitKey(radar.getCycleDuration()*1000);
        window.removeAllWidgets();
    }
    
      
    return 0;
}

