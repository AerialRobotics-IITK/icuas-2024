#include <bits/stdc++.h> 
using namespace std; 

#define MAXN 100 
#define INF 1e7; 

struct node {
	double x;
	double y;
	double z;
};

class trajectory_generator{
private:
    vector<struct node*> nodes;
    vector<vector<int>> all;

    int min_imum=INF;
    vector<int> minimum_vector;
    vector<vector<int>> minimums;
    vector<vector<double>> waypoint_matrix;

    int find_distance(int point1, int point2);
    void create_graph(double x, double y,double z);
    void get_all_points(int* possible,int count,vector<int> &store);
    int check_path(vector<int> path,int start_location);
    void permutations(vector<int> nums, int l, int h,int start_location);
    void permute(vector<int>& nums,int start_location);
    vector<int> find_shortest_permutation(vector<int> points, int start_location);
	void find_permutations();
	int find_next_plane_point(vector<int> plane0,vector<int> plane1);
public:
    trajectory_generator();
    ~trajectory_generator();
};

	

void get_waypoints(double inter_plant_dist){
	
	for(int i=0;i<nodes.size();i++){
		vector<double> waypoint1;
		waypoint1.push_back(nodes[i]->x-inter_plant_dist);waypoint1.push_back(nodes[i]->y);waypoint1.push_back(nodes[i]->z);
		
		vector<double> waypoint2;
		waypoint2.push_back(nodes[i]->x);waypoint2.push_back(nodes[i]->y);waypoint2.push_back(nodes[i]->z);

		vector<double> waypoint3;
		waypoint3.push_back(nodes[i]->x+inter_plant_dist);waypoint3.push_back(nodes[i]->y);waypoint3.push_back(nodes[i]->z);
		
		waypoint_matrix.push_back(waypoint1);
		waypoint_matrix.push_back(waypoint2);
		waypoint_matrix.push_back(waypoint3);
	}
}

void calculate_front(double distance){
	int length = waypoint_matrix.size();
	for(int i=0;i<length;i++){
		vector<double> waypoint = waypoint_matrix[i];
		waypoint_matrix.push_back(waypoint);
	}

	for(int i=0;i<length;i++){
		waypoint_matrix[i][0] = waypoint_matrix[i][0] - distance; 
	}													

	for(int i=length;i<2*length;i++){
		waypoint_matrix[i][0] = waypoint_matrix[i][0] + distance; 
	}
}

double calculate_waypoint_distance(vector<double> point1,vector<double> point2){
	double sum=0;
	for(int i=0;i<point1.size();i++){
		sum = sum + (point1[i]-point2[i])*(point1[i]-point2[i]);
	}
	return sum;
}


vector<vector<double>> translation(int shelf,vector<double> last_waypoint){

	vector<vector<double>> return_waypoints;
	vector<double> temp;
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


vector<vector<double>> convert_vector_to_waypoints(vector<int> shelves){
	vector<vector<double>> final_waypoints;
	vector<vector<double>> temp;

	for(int i=0;i<shelves.size();i++){
		if(i==0){
			vector<double> empty={};
			temp = translation(shelves[i],empty);

		}else{
			temp = translation(shelves[i],final_waypoints[final_waypoints.size()-1]);
		}


		for(int j=0;j<temp.size();j++){
			final_waypoints.push_back(temp[j]);
		}
	}

	return final_waypoints;
}

int main() 
{ 
	vector<int> points={1,5,2,6,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27};
	
	sort(points.begin(),points.end());

	// check_closest(points);
	vector<vector<int>> plane(3);

	for(int i = 0;i<points.size();i++){
		int plane_no = (points[i]-1)/9; 
		plane[plane_no].push_back(points[i]);
	}
	for(int i=0;i<plane.size();i++){
		cout << "Plane "<< i+1 << ": ";
		for(int j=0;j<plane[i].size();j++){
			cout<<plane[i][j]<<" ";
		}
		cout<<endl;
	}

	vector<int> final_path;

	create_graph(1,1,10);

	vector<int> plane0 = find_shortest_permutation(plane[0],0);


	for(int i =0;i<plane0.size();i++){
		final_path.push_back(plane0[i]);
	}

	for(int i=0;i<plane0.size();i++){
		final_path.push_back(-1*plane0[plane0.size()-1-i]);
	}

	int next = find_next_plane_point(plane0,plane[1]);

	min_imum = INF;

	vector<int> plane1 = find_shortest_permutation(plane[1],next);
	

	for(int i =0;i<plane1.size();i++){
		final_path.push_back(plane1[i]);
	}

	for(int i=0;i<plane1.size();i++){
		final_path.push_back(-1*plane1[plane1.size()-1-i]);
	}

	next = find_next_plane_point(plane1,plane[2]);


	min_imum = INF;

	vector<int> plane2 = find_shortest_permutation(plane[2],next);

	for(int i =0;i<plane2.size();i++){
		final_path.push_back(plane2[i]);
	}

	for(int i=0;i<plane2.size();i++){
		final_path.push_back(-1*plane2[plane2.size()-1-i]);
	}

	cout << endl;
	cout << "Final index of shelf in order: ";
	for(int i=0;i<final_path.size();i++){
		cout<<final_path[i]<<" ";
	}

	get_waypoints(7.0);
	calculate_front(6);
	vector<vector<double>> final_waypoints = convert_vector_to_waypoints(final_path);
	cout<<endl<<endl<< "Total no of points with plant:" <<final_path.size();

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

	cout << endl << endl;

	for(int i =0; i<final_waypoints.size(); i++){
		cout << "Point Coordinate " << i+1 << ": ";
		for(int j=0; j<final_waypoints[i].size(); j++){
			cout << " " << final_waypoints[i][j] << " ";
		}
		cout << endl;
	}

	cout << endl;

	return 0; 
}
