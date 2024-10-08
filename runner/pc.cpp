#include "mesh_controller.h"
#include "mesh_stream_builder.h"
#include "p2p_unsecured_short_interface.h"
#include "platform/p2p/win32_p2p.h"

using namespace MeshProto;


int main() {
    auto controller = new MeshController("dev network", 7768);
    controller->set_psk_password("1234");
    controller->callbacks.on_data_packet = [](MeshProto::far_addr_t src_addr, const ubyte* data, ushort size) {
        printf("Hello packet!\n");
        fflush(stdout);

        // sending response. builders are used to send large amount of data using many small chunks
        printf("%s\n", data);
        fflush(stdout);
    };

    Win32Serial serial(R"(\\.\COM3)", 115'200);
    P2PUnsecuredShortInterface uart_interface(true, false, serial, serial);
    Os::sleep_milliseconds(1000);
    controller->add_interface(&uart_interface);

    if (Os::random_u32() % 3) {
        printf("will send a data packet!\n");
        Os::sleep_milliseconds(5000);
        printf("sending a data packet\n");
        fflush(stdout);

        far_addr_t dst_addr = 0;
        for (auto& [far, peer] : controller->router.peers)
            dst_addr = far;
        dst_addr = BROADCAST_FAR_ADDR;

        auto lorem = "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Vivamus sodales euismod dolor. Maecenas "
                     "condimentum erat urna, vel consequat arcu hendrerit sit amet. Curabitur justo nunc, euismod id "
                     "nisi a, tincidunt eleifend ante. Vestibulum nec justo vel nisi consequat condimentum. Curabitur "
                     "nec nulla ac orci tempus commodo. Donec molestie euismod ante, in efficitur lorem posuere et. "
                     "Curabitur euismod eleifend lectus et vestibulum. Nunc non mauris id leo tristique sollicitudin "
                     "a eget turpis.";

        MeshStreamBuilder builder(*controller, dst_addr, strlen(lorem) + 1);
        builder.write((ubyte*) lorem, strlen(lorem) + 1);
    }

    Os::sleep_milliseconds(-1);

    return 0;
}