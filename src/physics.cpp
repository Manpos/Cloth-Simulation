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

float separationX = 1, separationY = 1;
float gravity = -9.8;
vec3 spherePosition (0.0f, 1.0f, 0.0f);
float sphereRadius = 1.0f;

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

void IsCollidingSphere(float radius, vec3 center, vec3 &position) {
	vec3 pointSphere(position.x - center.x, position.y - center.y, position.z - center.z);
	if (sqrt((pointSphere.x * pointSphere.x) + (pointSphere.y * pointSphere.y) + (pointSphere.z * pointSphere.z)) <= radius) {

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

	return -(e * (length(position[part1] - position[part2]) - L) + dot(d * (velocity[part1] - velocity[part2]), ((position[part1] - position[part2]) / length(position[part1] - position[part2])))) * 
		((position[part1] - position[part2]) / length(position[part1] - position[part2]));
}


void springsForceCalc(vec3* position, vec3* velocity, vec3* resultForce, int i, int rows, int cols, float L, float d, float e) {
	vec3 totalForce = clothVertexForce[i];
	//Strech 
	//Horizontal
	//std::cout << i << std::endl;
	if (i - 1 * cols >= 0) {
		//std::cout << "NO LEFT LIMIT" << std::endl;
		totalForce += forceBetweenParticles(position, velocity, i, i - 1 * cols, L, d, e);
	}
	if (i + 1 * cols <= rows * cols - 1) {
		//std::cout << "NO RIGHT LIMIT" << std::endl;
		totalForce += forceBetweenParticles(position, velocity, i, i + 1 * cols, L, d, e);
		//std::cout << totalForce.x << std::endl;
	}

	//Vertical
	if ((i % cols) - 1 >= 0) {
		//std::cout << "NO UP LIMIT" << std::endl;
		totalForce += forceBetweenParticles(position, velocity, i, (i % cols) - 1, L, d, e);
	}
	if ((i % cols) + 1 <= cols - 1) {
		//std::cout << "NO DOWN LIMIT" << std::endl;
		totalForce += forceBetweenParticles(position, velocity, i, (i % cols) + 1, L, d, e);
	}

	//Diagonal
	//Up-left
	if (i - 1 * cols >= 0 && (i % cols) - 1 >= 0) {
		//std::cout << "UP-LEFT PARTICLE AVAILABLE" << std::endl;
		totalForce += forceBetweenParticles(position, velocity, i, i - 1 * cols - 1, sqrt(L*L + L * L), d, e);
	}
	
	//Up-right
	if (i + 1 * cols <= rows * cols - 1 && (i % cols) - 1 >= 0) {
		//std::cout << "UP-RIGHT PARTICLE AVAILABLE" << std::endl;
		totalForce += forceBetweenParticles(position, velocity, i, i + 1 * cols - 1, sqrt(L*L + L * L), d, e);
	}

	//Down-left
	if (i - 1 * cols >= 0 && (i % cols) + 1 <= cols - 1) {
		//std::cout << "DOWN-LEFT PARTICLE AVAILABLE" << std::endl;
		totalForce += forceBetweenParticles(position, velocity, i, i - 1 * cols + 1, sqrt(L*L + L * L), d, e);
	}
	//Down-right
	if (i + 1 * cols <= rows * cols - 1 && (i % cols) + 1 <= cols - 1) {
		//std::cout << "DOWN-RIGHT PARTICLE AVAILABLE" << std::endl;
		totalForce += forceBetweenParticles(position, velocity, i, i + 1 * cols + 1, sqrt(L*L + L * L), d, e);
	}
	
	//Bending springs
	//Horizontal
	//std::cout << i << std::endl;
	if (i - 2 * cols >= 0) {
		//std::cout << "NO LEFT LIMIT" << std::endl;
		totalForce += forceBetweenParticles(position, velocity, i, i - 2 * cols, L * 2, d, e);

		//std::cout << totalForce.x << std::endl;
	}
	if (i + 2 * cols <= rows * cols - 1) {
		//std::cout << "NO RIGHT LIMIT" << std::endl;
		totalForce += forceBetweenParticles(position, velocity, i, i + 2 * cols, L * 2, d, e);
	}

	//Vertical
	if ((i % cols) - 2 >= 0) {
		//std::cout << "NO UP LIMIT" << std::endl;
		totalForce += forceBetweenParticles(position, velocity, i, (i % cols) - 2, L * 2, d, e);
	}
	if ((i % cols) + 2 <= cols - 1) {
		//std::cout << "NO DOWN LIMIT" << std::endl;
		totalForce += forceBetweenParticles(position, velocity, i, (i % cols) + 2, L * 2, d, e);
	}
	//std::cout << totalForce.x << std::endl;
	totalForce += vec3(0, gravity, 0);
	resultForce[i] = totalForce;
}

void verletSolver(vec3 &position, vec3 &prevPosition, vec3 &velocity, vec3 force, float m, float time) {
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
	clothVertexPosition = CreateClothMeshArray(ClothMesh::numRows, ClothMesh::numCols, 0.5, 0.5, meshPosition);
	clothVertexPrevPosition = new vec3[ClothMesh::numVerts];
	clothVertexVelocity = new vec3[ClothMesh::numVerts];
	clothVertexForce = new vec3[ClothMesh::numVerts];

	clothVertexVelocity[0] = vec3(0, 0, 0);
	clothVertexVelocity[13] = vec3(0, 0, 0);
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
			springsForceCalc(clothVertexPosition, clothVertexVelocity, clothVertexForce, i, ClothMesh::numRows, ClothMesh::numCols, separationX, 0.7, 0.7);
			IsCollidingBox(planeDown, clothVertexPrevPosition[i], clothVertexPosition[i]);
		}
		
	}

	ClothMesh::updateClothMesh(&clothVertexPosition[0].x);
}
void PhysicsCleanup() {
	//TODO
}