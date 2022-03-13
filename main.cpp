#include <iostream>
#include "lvx_file.h"
#include "lds_lvx.h"
int main(int argc, char **argv) {

    if (argc != 3 )
    {
        std::cerr << "usage \n";
        std::cerr << argv[0] <<" sample.lvx sample.csv "<<std::endl;
        return 0;
    }
    const  std::string input_file (argv[1]);
    const std::string output_file (argv[2]);
    std::cout <<"Loading file " << input_file << std::endl;

    livox_ros::LvxFileHandle lvx_file;
    lvx_file.Open(input_file.c_str(), std::ios::in);

    if (lvx_file.GetFileState() != livox_ros::kLvxFileOk){
        std::cerr << "Problem with lvx " << lvx_file.GetFileState() << std::endl;
    }
    livox_ros::OutPacketBuffer packets_of_frame;
    const uint32_t kMaxPacketsNumOfFrame = 8192;
    packets_of_frame.buffer_capacity =kMaxPacketsNumOfFrame * sizeof(livox_ros::LvxFilePacket);
    packets_of_frame.data_size =0;
    packets_of_frame.packet =new uint8_t[kMaxPacketsNumOfFrame * sizeof(livox_ros:: LvxFilePacket)];

    std::ofstream txt (output_file.c_str());
    int points = 0;
    int r = 0;
    while (r == livox_ros::kLvxFileOk)
    {
        const auto progress = lvx_file.GetLvxFileReadProgress();
        std::cout << progress << std::endl;
        r = lvx_file.GetPacketsOfFrame(&packets_of_frame);
        if (!r){
            uint32_t data_size = packets_of_frame.data_size;
            uint8_t *packet_base = packets_of_frame.packet;
            uint32_t data_offset = 0;
            while (data_offset < data_size) {
                LivoxEthPacket *eth_packet;
                int32_t handle;
                uint8_t data_type;
                if (lvx_file.GetFileVersion()) {
                    livox_ros::LvxFilePacket *detail_packet =
                            (livox_ros::LvxFilePacket *)&packet_base[data_offset];
                    eth_packet = (LivoxEthPacket *)(&detail_packet->version);
                    handle = detail_packet->device_index;
                } else {
                    livox_ros::LvxFilePacketV0 *detail_packet =
                            (livox_ros::LvxFilePacketV0 *)&packet_base[data_offset];
                    eth_packet = (LivoxEthPacket *)(&detail_packet->version);
                    handle = detail_packet->device_index;
                }

                data_type = eth_packet->data_type;
                int packet_ponts = 0;
                if (data_type == kExtendCartesian)
                {
                    const int points_in_packet = livox_ros::GetPointsPerPacket(eth_packet->data_type);
                    size_t packet_offset = 0;
                    for (int i=0; i < points_in_packet; i++) {
                        LivoxExtendRawPoint *point = reinterpret_cast<LivoxExtendRawPoint *>(eth_packet->data+packet_offset);
                        points++;
                        packet_ponts++;
                        packet_offset+=sizeof(LivoxExtendRawPoint);
                        txt << point->x <<" "<< point->y <<" "<< point->z <<" " << (int) point->reflectivity <<std::endl;
                    }
                }
                data_offset += (livox_ros::GetEthPacketLen(data_type) + 1);
            }
        }
    }
    txt.close();
    std::cout << "lvx finished with " << r << std::endl;
    std::cout << " points " << points << std::endl;
    delete packets_of_frame.packet;
    return 0;
}
