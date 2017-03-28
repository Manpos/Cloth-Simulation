#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <glm\glm.hpp>

bool show_test_window = false;
using namespace glm;

namespace ClothMesh {
	extern const int numCols;
	extern const int numRows;
	extern const int numVerts;
	extern void updateClothMesh(float* array_data);
}

vec3 *clothVertexPosition;
vec3 *clothVertexPrevPosition;
vec3 *clothVertexVelocity;
vec3 *clothVertexForce;

float separationX = 1, separationY = 1;

struct Plane
{
	vec3 normal;
	float d;
};

	// Planes of the cube
	Plane planeDown, planeLeft, planeRight, planeFront, planeBack, planeTop;

	//  Dot product by passing a vector and a position (three floats) 
	float DotProduct(vec3 vector, float x1, float y1, float z1) {
		return ((vector.x * x1) + (vector.y * y1) + (vector.z * z1));
	}

	//// Calculates if the cloth is traspassing a plane
	//void planeCollision(int i, Plane plane) {
	//	if (((DotProduct(plane.normal, clothVertexPrevPosition[i * 3], clothVertexPrevPosition[i * 3 + 1], clothVertexPrevPosition[i * 3 + 2]) + plane.d)*
	//		(DotProduct(plane.normal, clothVertexPosition[i * 3], clothVertexPosition[i * 3 + 1], clothVertexPosition[i * 3 + 2]) + plane.d)) <= 0) {
	//		ImGui::Text("Bomb has been planted");
	//	}
	//}

void GUI() {
	{	//FrameRate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		//TODO
	}

	// ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

vec3 * CreateClothMeshArray(int rowVerts, int columnVerts, float vertexSeparationX, float vertexSeparationY, vec3 position) {
	int currPosX = 0, currPosY = 0;
	vec3 *result = new vec3[columnVerts * rowVerts * 3];
	for (int i = 0; i < rowVerts * columnVerts; ++i) {

		if ( currPosX < columnVerts) {
			result[i] = vec3 (position.x + vertexSeparationY * currPosY, position.y, position.z + vertexSeparationY * currPosX);
		}

		currPosX++;

		if(currPosY < rowVerts && currPosX >= columnVerts){
			currPosX = 0;
			currPosY++;
		}
	}
	return result;
}


void PhysicsInit() {
	vec3 meshPosition ( -4 , 8, -4 );
	clothVertexPosition = CreateClothMeshArray(ClothMesh::numRows, ClothMesh::numCols, 0.5, 0.5, meshPosition);
	clothVertexPrevPosition = new vec3[ClothMesh::numVerts];
	clothVertexVelocity = new vec3[ClothMesh::numVerts];
	clothVertexForce = new vec3[ClothMesh::numVerts];

	for (int i = 0; i < ClothMesh::numVerts; ++i) {

		clothVertexPrevPosition[i] = clothVertexPosition[i];
		clothVertexVelocity[i] = vec3(0, 0, 0);
	}

	// Fill the plane sides with the equations
	planeDown = { vec3(0.f, 1.f, 0.f), 0.f };
	planeTop = { vec3(0.f, -1.0f, 0.f), 10.0f };
	planeLeft = { vec3(1.f, 0.0f, 0.f), 5.f };
	planeRight = { vec3(-1.f, 0.0f, 0.f), 5.f };
	planeBack = { vec3(0.f, 0.0f, 1.f), 5.f };
	planeFront = { vec3(0.f, 0.0f, -1.f), 5.f };

	//TODO
}
void PhysicsUpdate(float dt) {
	//TODO
	for (int i = 0; i < ClothMesh::numVerts; ++i) {

		vec3 temp;
		vec3 vertForce;
		vec3 nextVertForce;

		if (i != 0 && i != 13) {

			//Verlet prev position save
			temp = clothVertexPosition[i];

			//Verlet position update X,Y,Z
			clothVertexPosition[i] = clothVertexPosition[i] + (clothVertexPosition[i] - clothVertexPrevPosition[i]) + vec3(0,-9.8, 0) * (dt*dt);

			//Verlet last position save X,Y,Z
			clothVertexPrevPosition[i] = temp;

			clothVertexVelocity[i] = (clothVertexPosition[i] - clothVertexPrevPosition[i]) / dt;


		}
		

	}

	ClothMesh::updateClothMesh(&clothVertexPosition[0].x);
}
void PhysicsCleanup() {
	//TODO
}