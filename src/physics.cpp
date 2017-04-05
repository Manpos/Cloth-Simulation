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
float stretchD = 2, shearD = 2, bendD = 2, stretchK = 100, shearK, bendK;

#pragma endregion


struct Plane
{
	vec3 normal;
	float d;
};

// Planes of the cube
Plane planeDown, planeLeft, planeRight, planeFront, planeBack, planeTop;

// Mathematic functions

	// Solve a quadratic equation to calculate a value of the point where collides the vector against the sphere
	float SphereQuadraticEquation(float a, float b, float c) {
		float x1 = -b + sqrt((b * b) - 4 * a * c) / (2 * a);
		float x2 = -b - sqrt((b * b) - 4 * a * c) / (2 * a);

		// We just need the value is between 0 and 1
		if (x1 >= 0 && x1 <= 1)
			return x1;
		if (x2 >= 0 && x2 <= 1)
			return x2;
	}

	// Calculate the collision point between two points and a sphere
	vec3 PointCollisionSphereVector(vec3 position, vec3 prevPosition, float sphereRadius, vec3 sphereCenter) {
		// Vector between positions (P1 - P0)
		vec3 vectorPos{ position.x - prevPosition.x, position.y - prevPosition.y, position.z - prevPosition.z };
		vectorPos = normalize(vectorPos);

		// Vector between the center of the sphere and P0
		vec3 vecSpherePos{ prevPosition.x - sphereCenter.x, prevPosition.y - sphereCenter.y, prevPosition.z - sphereCenter.z };

		//Calculate the three components of the quadratic equation (ax2 + bx + c = 0)
		// a = (P1 - P0)^2
		float a = dot(vectorPos, vectorPos);
		// b = 2 * ((P1 - P0) * (P0 - Sc))
		float b = 2 * dot(vectorPos, vecSpherePos);
		// c = (P0 - Sc) * (P0 - Sc) - Sr^2
		float c = dot(vecSpherePos, vecSpherePos) - (sphereRadius * sphereRadius);

		// Result of the quadratic equation
		float alpha = SphereQuadraticEquation(a, b, c);

		// Calculate the point of the collision
		return prevPosition + (position - prevPosition) * alpha;
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
		vec3 point = PointCollisionSphereVector(position, prevPos, radius, center);
		
		std::cout << "P1	" << position.x << " " << position.y << " " << position.z << std::endl;
		std::cout << "Collision point	" << point.x << " " << point.y << " " << point.z << std::endl;
		std::cout << "P0	" << prevPos.x << " " << prevPos.y << " " << prevPos.z << std::endl << std::endl;
		
	}
}

void GUI() {
	{	//FrameRate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::InputFloat("Constant", &stretchK, 0, 1000);
		ImGui::InputFloat("Stretch Damping", &stretchD, 0, 100);
		ImGui::InputFloat("Shear Damping", &shearD, 0, 100);
		ImGui::InputFloat("Bend Damping", &bendD, 0, 100);
	}

	// ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
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

	return -(e * (length(position[part1] - position[part2]) - L) + dot(d *(velocity[part1] - velocity[part2]), ((position[part1] - position[part2]) / length(position[part1] - position[part2])))) *
		((position[part1] - position[part2]) / length(position[part1] - position[part2]));
}


void springsForceCalc(vec3* position, vec3* velocity, vec3* resultForce, int i, int rows, int cols, float L, float stretchd, float sheard, float bendd , float e) {
	
	vec3 totalForce = vec3(0.f);

	//Strech 

	//LEFT
	if ((i / cols) - 1 >= 0) {
		totalForce += forceBetweenParticles(position, velocity, i, i - 1 * cols, L, stretchd, e); }

	//RIGHT
	if ((i / cols) + 1 <= rows - 1) { 
		totalForce += forceBetweenParticles(position, velocity, i, i + 1 * cols, L, stretchd, e); }

	//UP
	if ((i % cols) - 1 >= 0) { 
		totalForce += forceBetweenParticles(position, velocity, i, i - 1, L, stretchd, e); }

	//DOWN
	if ((i % cols) + 1 <= cols - 1) { 
		totalForce += forceBetweenParticles(position, velocity, i, i + 1, L, stretchd, e); }

	//Shear

	//UP-LEFT
	if ((i / cols) - 1 >= 0 && (i % cols) - 1 >= 0) { 
		totalForce += forceBetweenParticles(position, velocity, i, i - 1 * cols - 1, sqrt(L * L + L * L), sheard, e); }
	
	//UP-RIGHT
	if ((i / cols) + 1 <= rows - 1 && (i % cols) - 1 >= 0) { 
		totalForce += forceBetweenParticles(position, velocity, i, i + 1 * cols - 1, sqrt(L * L + L * L), sheard, e); }

	//DOWN-LEFT
	if ((i / cols) - 1 >= 0 && (i % cols) + 1 <= cols - 1) { 
		totalForce += forceBetweenParticles(position, velocity, i, i - 1 * cols + 1, sqrt(L * L + L * L), sheard, e); }

	//DOWN-RIGHT
	if ((i / cols) + 1 <= rows - 1 && (i % cols) + 1 <= cols - 1) {	
		totalForce += forceBetweenParticles(position, velocity, i, i + 1 * cols + 1, sqrt(L * L + L * L), sheard, e); }
	
	//Bending springs

	//LEFT
	if ((i / cols) - 2 >= 0) { 
		totalForce += forceBetweenParticles(position, velocity, i, i - 2 * cols, L * 2, bendd, e); }

	//RIGHT
	if ((i / cols) + 2 <= rows - 1) { 
		totalForce += forceBetweenParticles(position, velocity, i, i + 2 * cols, L * 2, bendd, e); }

	//UP
	if ((i % cols) - 2 >= 0) { 
		totalForce += forceBetweenParticles(position, velocity, i, i - 2, L * 2, bendd, e); }

	//DOWN
	if ((i % cols) + 2 <= cols - 1) { 
		totalForce += forceBetweenParticles(position, velocity, i, i + 2, L * 2, bendd, e); }

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
	if (ImGui::Button("Reset")) { PhysicsInit(); };

	for (int i = 0; i < ClothMesh::numVerts; ++i) {

		if (i != 0 && i != 13) {

			verletSolver(clothVertexPosition[i], clothVertexPrevPosition[i], clothVertexVelocity[i], clothVertexForce[i], 1, dt);
			//IsCollidingBox(planeDown, clothVertexPrevPosition[i], clothVertexPosition[i]);
			//IsCollidingSphere(sphereRadius, spherePosition, clothVertexPosition[i], clothVertexPrevPosition[i]);
			springsForceCalc(clothVertexPosition, clothVertexVelocity, clothVertexForce, i, ClothMesh::numRows, ClothMesh::numCols, separationX, stretchD, shearD, bendD, stretchK);
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