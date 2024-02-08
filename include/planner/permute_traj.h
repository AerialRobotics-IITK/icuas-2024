
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <map>
#include <memory>

#define DEBUG 1

namespace trajectory_gen
{
	struct node{
		double x;
		double y;
		double z;
	};
}

class trajectory_gen{
	private:
		const int INF = 1e7; 
		int min_imum=INF;
		double x_origin;
		double y_origin;
		double z_origin;

		std::vector<struct node*> vertices;
		std::vector<std::vector<int>> all;
		std::vector<int> minimum_vector;
		std::vector<std::vector<int>> minimums;

		std::vector<std::vector<double>> waypoint_matrix;
		std::vector<int> points;
		std::vector<int> final_path;
		std::vector<std::vector<double>> final_waypoints;

	public:

		void create_graph(double x, double y,double z){

			struct node* temp_node =  new struct node;
			temp_node->x = 0; temp_node->y = 0; temp_node->z = 0;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 0; temp_node->y = 0; temp_node->z = z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 0; temp_node->y = 0; temp_node->z = 2*z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 0; temp_node->y = y; temp_node->z = 0;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 0; temp_node->y = y; temp_node->z = z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 0; temp_node->y = y; temp_node->z = 2*z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 0; temp_node->y = 2*y; temp_node->z = 0;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 0; temp_node->y = 2*y; temp_node->z = z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 0; temp_node->y = 2*y; temp_node->z = 2*z;
			vertices.push_back(temp_node);



			temp_node =  new struct node;
			temp_node->x = x; temp_node->y = 0; temp_node->z = 0;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = x; temp_node->y = 0; temp_node->z = z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = x; temp_node->y = 0; temp_node->z = 2*z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = x; temp_node->y = y; temp_node->z = 0;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = x; temp_node->y = y; temp_node->z = z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = x; temp_node->y = y; temp_node->z = 2*z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = x; temp_node->y = 2*y; temp_node->z = 0;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = x; temp_node->y = 2*y; temp_node->z = z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = x; temp_node->y = 2*y; temp_node->z = 2*z;
			vertices.push_back(temp_node);



			temp_node =  new struct node;
			temp_node->x = 2*x; temp_node->y = 0; temp_node->z = 0;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 2*x; temp_node->y = 0; temp_node->z = z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 2*x; temp_node->y = 0; temp_node->z = 2*z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 2*x; temp_node->y = y; temp_node->z = 0;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 2*x; temp_node->y = y; temp_node->z = z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 2*x; temp_node->y = y; temp_node->z = 2*z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 2*x; temp_node->y = 2*y; temp_node->z = 0;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 2*x; temp_node->y = 2*y; temp_node->z = z;
			vertices.push_back(temp_node);

			temp_node =  new struct node;
			temp_node->x = 2*x; temp_node->y = 2*y; temp_node->z = 2*z;
			vertices.push_back(temp_node);
		}

		int find_distance(int point1, int point2){
			struct node node1;
			struct node node2;

			if(point1 ==0){
				node1.x = -1;
				node1.y = -1;
				node1.z = 0;
			}else{
				node1 = *vertices[point1-1];
			}	
			node2 = *vertices[point2-1];

			return (node1.x - node2.x)*(node1.x - node2.x) + (node1.y - node2.y)*(node1.y - node2.y) + (node1.z - node2.z)*(node1.z - node2.z);
		}

		void get_all_points(int* possible,int count,std::vector<int> &store){
			if(count == 9){
				std::vector<int> hello;
				for(int i =0;i<store.size();i++){
					if(store[i]!=0){
						hello.push_back(store[i]);
					}
				}
				all.push_back(hello);
				return;
			}
			store[count] = 0;
			get_all_points(possible,count+1,store);	
			store[count] = possible[count];
			get_all_points(possible,count+1,store);
		}

		int check_path(std::vector<int> path,int start_location){
			double sum=0;
			for(int i=0;i<path.size();i++){
				double val;
				if(i==0){
					val = find_distance(start_location,path[i]);
				}else{
					val = find_distance(path[i],path[i-1]);
				}sum = sum+val;
			}
			return sum;
		}

		void permutations(std::vector<int> nums, int l, int h,int start_location) 
		{ 	
			if (l == h){ 
				int tmp =check_path(nums,start_location);
				if(tmp<min_imum){
					minimum_vector = nums;
					min_imum = tmp;
				}		
				return; 
			} 

			for (int i = l; i <= h; i++) { 
				std::swap(nums[l], nums[i]);   
				permutations(nums, l + 1, h,start_location); 
				std::swap(nums[l], nums[i]); 
			} 
		} 
		
		void permute(std::vector<int>& nums,int start_location) 
		{ 
			permutations(nums, 0,nums.size()-1,start_location); 
		} 

		std::vector<int> find_shortest_permutation(std::vector<int> points, int start_location){
				std::vector<int> min_vec;
				if(points.size()>2){
					permute(points,start_location);
					min_vec = minimum_vector;
				}
			return min_vec;	
		}

		void find_permutations(){
			int possible[9] ={1,2,3,4,5,6,7,8,9};
			std::vector<int> store(9);
			get_all_points(possible,0,store);
		}

		int find_next_plane_point(std::vector<int> plane0,std::vector<int> plane1){
			int min_dist=INF;
			int next=0;
			for(int i=0;i<plane1.size();i++){
				if(plane0.empty()){
					break;
				}
				int tmp = find_distance(plane1[i],plane0[0]);
				if(min_dist>tmp){
					min_dist = find_distance(plane1[i],plane0[0]);
					next = plane1[i];
				}
			}
			return next;
		}
			
		void get_waypoints(double inter_plant_dist){
			
			for(int i=0;i<vertices.size();i++){
				std::vector<double> waypoint1;
				waypoint1.push_back(vertices[i]->x);waypoint1.push_back(vertices[i]->y-inter_plant_dist);waypoint1.push_back(vertices[i]->z);
				
				std::vector<double> waypoint2;
				waypoint2.push_back(vertices[i]->x);waypoint2.push_back(vertices[i]->y);waypoint2.push_back(vertices[i]->z);

				std::vector<double> waypoint3;
				waypoint3.push_back(vertices[i]->x);waypoint3.push_back(vertices[i]->y+inter_plant_dist);waypoint3.push_back(vertices[i]->z);
				
				waypoint_matrix.push_back(waypoint1);
				waypoint_matrix.push_back(waypoint2);
				waypoint_matrix.push_back(waypoint3);
			}
		}

		void calculate_front(double distance){
			int length = waypoint_matrix.size();
			for(int i=0;i<length;i++){
				std::vector<double> waypoint = waypoint_matrix[i];
				waypoint_matrix.push_back(waypoint);
			}

			for(int i=0;i<length;i++){
				waypoint_matrix[i][0] = waypoint_matrix[i][0] - distance; 
			}													

			for(int i=length;i<2*length;i++){
				waypoint_matrix[i][0] = waypoint_matrix[i][0] + distance; 
			}
		}

		double calculate_waypoint_distance(std::vector<double> point1,std::vector<double> point2){
			double sum=0;
			for(int i=0;i<point1.size();i++){
				sum = sum + (point1[i]-point2[i])*(point1[i]-point2[i]);
			}
			return sum;
		}

		std::vector<std::vector<double>> translation(int shelf,std::vector<double> last_waypoint){
			std::vector<std::vector<double>> return_waypoints;
			std::vector<double> temp;
			if(shelf<0){
				for(int i=-1;i<2;i++){
					temp = waypoint_matrix[3*(-1*shelf-1)+1+i+waypoint_matrix.size()/2];
					return_waypoints.push_back(temp);
				}
			
			}else{
				for(int i=-1;i<2;i++){
					temp = waypoint_matrix[3*(shelf-1)+1+i];
					return_waypoints.push_back(temp);
				}
			}
			
			if(last_waypoint.empty()){
				return return_waypoints;
			}else{
				double distance_zero = calculate_waypoint_distance(last_waypoint,return_waypoints[0]);
				double distance_one = calculate_waypoint_distance(last_waypoint,return_waypoints[2]);
				if(distance_zero>distance_one){
					reverse(return_waypoints.begin(),return_waypoints.end());
					return return_waypoints;
				}else{
					return return_waypoints;
				}
			}
		}

		std::vector<std::vector<double>> convert_vector_to_waypoints(std::vector<int> shelves){
			std::vector<std::vector<double>> final_waypoints;
			std::vector<std::vector<double>> temp;

			for(int i=0;i<shelves.size();i++){
				if(i==0){
					std::vector<double> empty={};
					temp = translation(shelves[i],empty);

				}else{
					temp = translation(shelves[i],final_waypoints[final_waypoints.size()-1]);
				}


				for(int j=0;j<temp.size();j++){
					final_waypoints.push_back(temp[j]);
				}
			}

			return final_waypoints;
		};

		trajectory_gen(double x, double y, double z, double x_offset, double y_offset, double z_offset, std::vector<int> Points){
			points = Points;
			
			sort(points.begin(),points.end());

			std::vector<std::vector<int>> plane(3);
			
			for(int i = 0;i<points.size();i++){
				int plane_no = (points[i]-1)/9; 
				plane[plane_no].push_back(points[i]);
			}

			// for(int i=0;i<plane.size();i++){
			// 	std::cout << "Plane "<< i+1 << ": ";
			// 	for(int j=0;j<plane[i].size();j++){
			// 		std::cout<<plane[i][j]<<" ";
			// 	}
			// 	std::cout<<std::endl;
			// }

			create_graph(x,y,z);
			
			if(plane[0].empty()){
			}

			std::vector<int> plane0 = find_shortest_permutation(plane[0],0);

			for(int i =0;i<plane0.size();i++){
				final_path.push_back(plane0[i]);
			}

			for(int i=0;i<plane0.size();i++){
				final_path.push_back(-1*plane0[plane0.size()-1-i]);
			}

			int next;
			int temp;
			if(temp = find_next_plane_point(plane0,plane[1])){
				next = temp;
			}else{
				next=temp;

			}

			min_imum = INF;

			std::vector<int> plane1 = find_shortest_permutation(plane[1],next);
			

			for(int i =0;i<plane1.size();i++){
				final_path.push_back(plane1[i]);
			}

			for(int i=0;i<plane1.size();i++){
				final_path.push_back(-1*plane1[plane1.size()-1-i]);
			}

			temp = find_next_plane_point(plane1,plane[2]);

			if(temp){
				next = temp;
			}else{
				if(plane1.empty()&&plane[2].empty()){
					next = temp;
				}else if(plane1.empty()){
					if(final_path.empty()){
						next = 0;
					}else{
						next = final_path[final_path.size()-1];
					}

				}else if(plane[2].empty()){
					if(final_path.empty()){
						next = 0;
					}else{
						next = final_path[final_path.size()-1];
					}
				}
			}

			min_imum = INF;

			std::vector<int> plane2 = find_shortest_permutation(plane[2],next);

			for(int i =0;i<plane2.size();i++){
				final_path.push_back(plane2[i]);
			}

			for(int i=0;i<plane2.size();i++){
				final_path.push_back(-1*plane2[plane2.size()-1-i]);
			}

			std::cout << std::endl;
			std::cout << "Final index of shelf in order: ";

			get_waypoints(1.5);
			calculate_front(3);
			final_waypoints = convert_vector_to_waypoints(final_path);
			
			for(int i = 0;i<final_path.size();i++){
				std::cout<<final_path[i]<<" ";
			}
			
			std::cout<<std::endl<<std::endl<< "Total no of points with plant:" <<final_path.size()<<std::endl;

			for(int i = 0; i < final_path.size(); i++){
				if(final_path[i] > 0){
					for(int k = 0; k<3; k++){
						final_waypoints[(3*i)+k].push_back(0);
					}
				}
				else{
					for(int k = 0; k<3; k++){
						final_waypoints[(3*i)+k].push_back(180);
					}
				}
			}

			x_origin = x_offset;
			y_origin = y_offset;
			z_origin = z_offset;


			for(int i=0;i<final_waypoints.size();i++){
				final_waypoints[i][0] = final_waypoints[i][0] + x_origin;
				final_waypoints[i][1] = final_waypoints[i][1] + y_origin;
			    final_waypoints[i][2] = final_waypoints[i][2] + z_origin;
			}


			for(int i =0; i<final_waypoints.size(); i++){
				std::cout << "Point Coordinate " << i+1 << ": ";
				for(int j=0; j<final_waypoints[i].size(); j++){
					std::cout << " " << final_waypoints[i][j] << " ";
				}
				std::cout << std::endl;
			}

		}

		std::vector<std::vector<double>> get_final_waypoints(){
			
			return final_waypoints;
		}

};