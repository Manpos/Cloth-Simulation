#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <glm\glm.hpp>
#include <iostream>

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

#pragma region Variables

float separationX = 0.5, separationY = 0.5;
float gravity = -9.8;
vec3 spherePosition (0.0f, 1.0f, 0.0f);
float sphereRadius = 1.0f;
vec3 fixedInitialPosition[2];

#pragma endregion


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

void Bounce(vec3 &prevPosition, vec3 &position, Plane plane) {
	position.y = 0;
	prevPosition.y = 0;
}

void IsCollidingBox(Plane plane, vec3 &prevPosition, vec3 &position) {
	if ((dot(plane.normal, prevPosition) + plane.d) * (dot(plane.normal, position) + plane.d) <= 0) {
		Bounce(prevPosition, position, plane);
	}
}

void IsCollidingSphere(float radius, vec3 center, vec3 &position, vec3 prevPos) {
	vec3 pointSphere(position.x - center.x, position.y - center.y, position.z - center.z);
	if (sqrt((pointSphere.x * pointSphere.x) + (pointSphere.y * pointSphere.y) + (pointSphere.z * pointSphere.z)) <= radius) {
		std::cout << position.x << " " << position.y << " " << position.z << std::endl;
		std::cout << prevPos.x << " " << prevPos.y << " " << prevPos.z << std::endl;

	}
}

vec3 * CreateClothMeshArray(int rowVerts, int columnVerts, float vertexSeparationX, float vertexSeparationY, vec3 position) {
	int currPosX = 0, currPosY = 0;
	vec3 *result = new vec3[columnVerts * rowVerts];
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

//Particle position vector array, particle velocity vector array, position in array (part1 & part2), original separation (L), damping (d), elasticity (e)
vec3 forceBetweenParticles(vec3* position, vec3* velocity, int part1, int part2, float L, float d, float e) {

	return -(e * (length(position[part1] - position[part2]) - L) + d * dot((velocity[part1] - velocity[part2]), ((position[part1] - position[part2]) / length(position[part1] - position[part2])))) * 
		((position[part1] - position[part2]) / length(position[part1] - position[part2]));
}


void springsForceCalc(vec3* position, vec3* velocity, vec3* resultForce, int i, int rows, int cols, float L, float d, float e) {
	
	vec3 totalForce = vec3(0.f);

	//Strech 

	//LEFT
	if ((i / cols) - 1 >= 0) { totalForce += forceBetweenParticles(position, velocity, i, i - 1 * cols, L, d, e); }

	//RIGHT
	if ((i / cols) + 1 <= rows - 1) { totalForce += forceBetweenParticles(position, velocity, i, i + 1 * cols, L, d, e); }

	//UP
	if ((i % cols) - 1 >= 0) { totalForce += forceBetweenParticles(position, velocity, i, i - 1, L, d, e); }

	//DOWN
	if ((i % cols) + 1 <= cols - 1) { totalForce += forceBetweenParticles(position, velocity, i, i + 1, L, d, e); }

	//Diagonal

	//UP-LEFT
	if ((i / cols) - 1 >= 0 && (i % cols) - 1 >= 0) { totalForce += forceBetweenParticles(position, velocity, i, i - 1 * cols - 1, sqrt(L * L + L * L), d, e); }
	
	//UP-RIGHT
	if ((i / cols) + 1 <= rows - 1 && (i % cols) - 1 >= 0) { totalForce += forceBetweenParticles(position, velocity, i, i + 1 * cols - 1, sqrt(L * L + L * L), d, e); }

	//DOWN-LEFT
	if ((i / cols) - 1 >= 0 && (i % cols) + 1 <= cols - 1) { totalForce += forceBetweenParticles(position, velocity, i, i - 1 * cols + 1, sqrt(L * L + L * L), d, e); }

	//DOWN-RIGHT
	if ((i / cols) + 1 <= rows - 1 && (i % cols) + 1 <= cols - 1) {	totalForce += forceBetweenParticles(position, velocity, i, i + 1 * cols + 1, sqrt(L * L + L * L), d, e); }
	
	//Bending springs

	//LEFT
	if ((i / cols) - 2 >= 0) { totalForce += forceBetweenParticles(position, velocity, i, i - 2 * cols, L * 2, d, e); }

	//RIGHT
	if ((i / cols) + 2 <= rows - 1) { totalForce += forceBetweenParticles(position, velocity, i, i + 2 * cols, L * 2, d, e); }

	//UP
	if ((i % cols) - 2 >= 0) { totalForce += forceBetweenParticles(position, velocity, i, i - 2, L * 2, d, e); }

	//DOWN
	if ((i % cols) + 2 <= cols - 1) { totalForce += forceBetweenParticles(position, velocity, i, i + 2, L * 2, d, e); }

	totalForce += vec3(0, gravity, 0);

	resultForce[i] = totalForce;
	
}

void verletSolver(vec3 &position, vec3 &prevPosition, vec3 &velocity, vec3 &force, float m, float time) {
	//Verlet prev position save
	vec3 temp = position;

	//Verlet position update X,Y,Z
	position = position + (position - prevPosition) + force/m * (time*time);

	//Verlet last position save X,Y,Z
	prevPosition = temp;

	velocity = (position - prevPosition) / time;
}

void PhysicsInit() {
	vec3 meshPosition ( -4 , 8, -4 );
	clothVertexPosition = CreateClothMeshArray(ClothMesh::numRows, ClothMesh::numCols, separationX, separationY, meshPosition);
	clothVertexPrevPosition = new vec3[ClothMesh::numVerts];
	clothVertexVelocity = new vec3[ClothMesh::numVerts];
	clothVertexForce = new vec3[ClothMesh::numVerts];

	for (int i = 0; i < ClothMesh::numVerts; ++i) {
		clothVertexPrevPosition[i] = clothVertexPosition[i];
		clothVertexVelocity[i] = vec3(0, 0, 0);
		clothVertexForce[i] = vec3(0, 0, 0);
	}

	// Fill the plane sides with the equations
	planeDown = { vec3(0.f, 1.f, 0.f), 0.f };
	planeTop = { vec3(0.f, -1.0f, 0.f), 10.0f };
	planeLeft = { vec3(1.f, 0.0f, 0.f), 5.f };
	planeRight = { vec3(-1.f, 0.0f, 0.f), 5.f };
	planeBack = { vec3(0.f, 0.0f, 1.f), 5.f };
	planeFront = { vec3(0.f, 0.0f, -1.f), 5.f };

}
void PhysicsUpdate(float dt) {
	//TODO
	for (int i = 0; i < ClothMesh::numVerts; ++i) {
		
		if (i != 0 && i != 13) {

			verletSolver(clothVertexPosition[i], clothVertexPrevPosition[i], clothVertexVelocity[i], clothVertexForce[i], 1, dt);
			IsCollidingBox(planeDown, clothVertexPrevPosition[i], clothVertexPosition[i]);
			springsForceCalc(clothVertexPosition, clothVertexVelocity, clothVertexForce, i, ClothMesh::numRows, ClothMesh::numCols, separationX, 2.5, 1);

		}


	}

	clothVertexPosition[14];
	clothVertexVelocity[14];
	clothVertexForce[14];

	ClothMesh::updateClothMesh(&clothVertexPosition[0].x);
}
void PhysicsCleanup() {
	//TODO
}