#include "lidar.h"
#include "globals.h"

int server_socket;
int client_socket;

void* lidar_start_receiving(void *arg) {
    server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket == -1) {
        printf("Socket creation failed\n");
        return 0;
    }
    sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(PORT);
    server_address.sin_addr.s_addr = inet_addr("127.0.0.1");
    socklen_t address_len = sizeof(server_address);
    
    int result = bind(server_socket, (struct sockaddr*)&server_address, sizeof(server_address));
    if (result < 0) {
        printf("Bind failed\n");
	    close(server_socket);
	    return 0;
    }
    result = listen(server_socket, 5);
    if (result < 0) {
        printf("Listen failed");
        close(server_socket);
	    return 0;
    }
    printf("Waiting for connection...\n");

    client_socket = accept(server_socket, (struct sockaddr*)&server_address, &address_len);
    if (client_socket < 0) {
    	printf("Accept failed\n");
	    close(client_socket);
	    return 0;
    }
    printf("Connected\n");
    lidar_data.lidar_connected = 1;

    return 0;
}

void* lidar_listen(void *arg) {
    printf("lidar_listen\n");
    char header[HEADER_SIZE];
    char data[DATA_BUFFER_SIZE];

    std::vector<Line> map_lines;
    map_lines.push_back(make_line(0,0, 3.600, 0));
    map_lines.push_back(make_line(3.600,0, 3.600, 2.400));
    map_lines.push_back(make_line(3.600,2.400, 0, 2.400));
    map_lines.push_back(make_line(0,2.400, 0, 0));

    std::vector<ScanPoint> scan;
    int prev_angle = 0;
    int has_begun_scan = 0;

    while (running) {
        ssize_t bytes_received = recv(client_socket, header, HEADER_SIZE, MSG_WAITALL);
        //printf("test\n");
	if (bytes_received != HEADER_SIZE) continue;
	
        if ((unsigned char)header[0] == 165) {
            int number_of_bytes_data = ((unsigned char)header[2] << 16) +
					((unsigned char)header[3] << 8) +
					(unsigned char)header[4];
	
	    if (number_of_bytes_data > DATA_BUFFER_SIZE) {
	    	printf("Data size received is to large!\n");
	        break;
	    }
	    
	    ssize_t data_received = recv(client_socket, data, number_of_bytes_data, MSG_WAITALL);
	    if (data_received != number_of_bytes_data){
		printf("Data size mismatch!\n");
		break;
	    }

	    int quality = ((unsigned char) data[0]) >> 2;
	    int angle = (((unsigned char) data[1]) >> 1) + (((unsigned char) data[2]) << 8) >> 7;
	    double distance = (((unsigned char) data[3]) + ((unsigned char) data[4] << 8) >> 2) / 1000.0;

	    double angle_rad = angle * (M_PI / 180.0f);
	    
	    if (quality == 0) continue;

            if (!lidar_data.is_scanning) {
                has_begun_scan = 0;
            }
        //printf("Quality %i, Angle %i, Angle rad %f, Distance %i\n", quality, angle, angle_rad, distance);

        if (angle < prev_angle) { // Perform scan match
            if (!has_begun_scan) {
                has_begun_scan = 1;
                continue;
            }

	    Pose2DWithCov res = scan_match(scan, map_lines, lidar_data.current_lidar_scan.pose);
	    if (res.pose.x != -1) { //invalid scan
	        lidar_data.current_lidar_scan.result = res;            

	        Pose2D pose = lidar_data.current_lidar_scan.result.pose;
                lidar_data.has_scanned = 1;
                lidar_data.is_scanning = 0;
	        lidar_data.has_scanned_once = 1;
	        has_begun_scan = 0;
#if 0
		    // Save scan to file
	   	FILE *file = fopen("scans.txt", "w");
	    	if (file) {
	             fprintf(file, "New scan:\n");
	             for (const auto& p : scan) {
	                  fprintf(file, "%.6f %.2f, %f\n", p.angle_rad * (180.0 / M_PI), p.distance, p.quality);
	             }
	             fprintf(file, "\n");
	             fclose(file);
	    	} else {
	             printf("Failed to open scans.txt for writing\n");
		}
#endif
	    } else { 
		printf("INVALID SCAN\n"); 
	    }
                 
            scan.clear();
        }
        ScanPoint point;
        point.angle_rad = angle_rad;
        point.distance = distance;
  	point.quality = quality;
        scan.push_back(point);
        prev_angle = angle;
        }
    }
    printf("Lidar listen ended...\n");
    return 0;
}

int lidar_running(int running) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) {
        printf("Socket creation failed");
	    return 1;
    }
    sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(PORT_START);
    server_address.sin_addr.s_addr = inet_addr("127.0.0.1");

    int result = connect(s, (struct sockaddr*)&server_address, sizeof(server_address));
    if (result < 0) {
	    printf("Connection failed\n");
	    close(s);
	    return 1;
    }

    //Send 2 bytes
    char message[2] = {0x0, 0x0};
    if (running == 1)
    	message[0] = 0x10;
    else
	message[0] = 0x20;

    if (send(s, message, sizeof(message), 0) != sizeof(message)) {
	printf("Send failed}\n");
	close(s);
	return 1;
    }
    if (running == 1)
    	printf("Start message sent.\n");
    else
	printf("Stop message sent.\n");
    close(s);
    return 0;
}

void lidar_shutdown() {
    close(client_socket);
    close(server_socket);
}