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

float separation = 0.5, nextSeparation = 0.5;
float gravity = -9.8;
vec3 spherePosition(2.0f, 3.0f, 0.0f);
float sphereRadius = 2.0f;
float stretchD = 40, shearD = 40, bendD = 40, stretchK = 1000, shearK = 1000, bendK = 1000;
float elasticity = 1;
float elongation = 5;
float timer = 0;
bool sphereCollision = true;
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
	vec3 vectorPos = normalize(position - prevPosition);

	// Vector between the center of the sphere and P0
	vec3 vecSpherePos = prevPosition - sphereCenter;

	//Calculate the three components of the quadratic equation (ax2 + bx + c = 0)
	// a = (P1 - P0)^2
	float a = 1;
	// b = 2 * ((P1 - P0) * (P0 - Sc))
	float b = 2 * dot(vectorPos, vecSpherePos);
	// c = (P0 - Sc) * (P0 - Sc) - Sr^2
	float c = dot(vecSpherePos, vecSpherePos) - (sphereRadius * sphereRadius);

	// Result of the quadratic equation
	float alpha = SphereQuadraticEquation(a, b, c);

	// Calculate the point of the collision
	return prevPosition + (position - prevPosition) * (alpha);
}


void Bounce(vec3 &prevPosition, vec3 &position, vec3 &velocity, Plane plane) {

	position = position - float(1 + elasticity) * (dot(plane.normal, position) + plane.d) * plane.normal;
	prevPosition = prevPosition - float(1 + elasticity) * (dot(plane.normal, prevPosition) + plane.d) * plane.normal;
	velocity = velocity - float(1 + elasticity) * dot(plane.normal, velocity) * plane.normal;

}

void IsCollidingBox(Plane plane, vec3 &prevPosition, vec3 &position, vec3 &velocity) {
	if ((dot(plane.normal, prevPosition) + plane.d) * (dot(plane.normal, position) + plane.d) <= 0) {
		Bounce(prevPosition, position, velocity, plane);
	}
}

void IsCollidingSphere(float radius, vec3 center, vec3 &position, vec3 &prevPos, vec3 &velocity) {
	vec3 pointSphere = position - center;
	if (length(pointSphere) <= radius) {
		Plane tanPlane;
		vec3 collisionPoint = PointCollisionSphereVector(position, prevPos, radius, center);
		tanPlane.normal = normalize(collisionPoint - center);
		tanPlane.d = -(dot(tanPlane.normal, collisionPoint));
		Bounce(prevPos, position, velocity, tanPlane);

	}
}

void GUI() {
	{	//FrameRate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::SliderFloat("Stretch Constant", &stretchK, 500, 1000);
		ImGui::SliderFloat("Shear Constant", &shearK, 500, 1000);
		ImGui::SliderFloat("Bend Constant", &bendK, 500, 1000);
		ImGui::SliderFloat("Stretch Damping", &stretchD, 10, 40);
		ImGui::SliderFloat("Shear Damping", &shearD, 10, 40);
		ImGui::SliderFloat("Bend Damping", &bendD, 10, 40);
		ImGui::SliderFloat("Elongation", &elongation, 5, 50);
		ImGui::SliderFloat("Points separation", &nextSeparation, 0.2, 0.5);
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

	return -(e * (length(position[part1] - position[part2]) - L) + 	d * dot((velocity[part1] - velocity[part2]), normalize(position[part1] - position[part2]))) * normalize(position[part1] - position[part2]);
}


void springsForceCalc(vec3* position, vec3* velocity, vec3* resultForce, int i, int rows, int cols, float L, float stretchd, float sheard, float bendd, float e1, float e2, float e3) {
	
	vec3 totalForce = vec3(0.f);

	//Strech 

	//LEFT
	if ((i / cols) - 1 >= 0) {
		totalForce += forceBetweenParticles(position, velocity, i, i - 1 * cols, L, stretchd, e1); }

	//RIGHT
	if ((i / cols) + 1 < rows) { 
		totalForce += forceBetweenParticles(position, velocity, i, i + 1 * cols, L, stretchd, e1); }

	//UP
	if ((i % cols) - 1 >= 0) { 
		totalForce += forceBetweenParticles(position, velocity, i, i - 1, L, stretchd, e1); }

	//DOWN
	if ((i % cols) + 1 < cols) { 
		totalForce += forceBetweenParticles(position, velocity, i, i + 1, L, stretchd, e1); }

	//Shear

	//UP-LEFT
	if ((i / cols) - 1 >= 0 && (i % cols) - 1 >= 0) { 
		totalForce += forceBetweenParticles(position, velocity, i, i - 1 * cols - 1, sqrt(L * L + L * L), sheard, e2); }
	
	//UP-RIGHT
	if ((i / cols) + 1 < rows && (i % cols) - 1 >= 0) { 
		totalForce += forceBetweenParticles(position, velocity, i, i + 1 * cols - 1, sqrt(L * L + L * L), sheard, e2); }

	//DOWN-LEFT
	if ((i / cols) - 1 >= 0 && (i % cols) + 1 < cols) { 
		totalForce += forceBetweenParticles(position, velocity, i, i - 1 * cols + 1, sqrt(L * L + L * L), sheard, e2); }

	//DOWN-RIGHT
	if ((i / cols) + 1 < rows && (i % cols) + 1 < cols) {	
		totalForce += forceBetweenParticles(position, velocity, i, i + 1 * cols + 1, sqrt(L * L + L * L), sheard, e2); }
	
	//Bending springs

	//LEFT
	if ((i / cols) - 2 >= 0) { 
		totalForce += forceBetweenParticles(position, velocity, i, i - 2 * cols, L * 2, bendd, e3); }

	//RIGHT
	if ((i / cols) + 2 < rows) { 
		totalForce += forceBetweenParticles(position, velocity, i, i + 2 * cols, L * 2, bendd, e3); }

	//UP
	if ((i % cols) - 2 >= 0) { 
		totalForce += forceBetweenParticles(position, velocity, i, i - 2, L * 2, bendd, e3); }

	//DOWN
	if ((i % cols) + 2 < cols) { 
		totalForce += forceBetweenParticles(position, velocity, i, i + 2, L * 2, bendd, e3); }

	totalForce += vec3(0, gravity, 0);

	resultForce[i] = totalForce;
	
}

void checkMaxElongation(vec3* position, vec3* velocity, int i, int rows, int cols, float L, float elongation) {

	//RIGHT
	if ((i / cols) + 1 < rows) {	
		if (length(position[i] - position[i + cols * 1]) > L + L * (elongation/100)) {
			float extraDistance = length(position[i] - position[i + cols * 1]) - (L + L * (elongation/100.f));
			vec3 aToB = normalize(position[i + cols * 1] - position[i]);
			vec3 bToA = normalize(position[i] - position[i + cols * 1]);
			if (velocity[i] == vec3(0.f)) {
				position[i + cols * 1] += bToA * extraDistance;
			}
			else if (velocity[i + cols * 1] == vec3(0.f)) {
				position[i] += aToB * extraDistance;
			}
			else {
				position[i] += aToB * extraDistance / 2.f;
				position[i + cols * 1] += bToA * extraDistance / 2.f;
			}			
		}
	}

	//DOWN
	if ((i % cols) + 1 < cols) {
	
		if (length(position[i] - position[i + 1]) > L + L * (elongation / 100)) {
			float extraDistance = length(position[i] - position[i + 1]) - (L + L*(elongation / 100));
			vec3 aToB = normalize(position[i + 1] - position[i]);
			vec3 bToA = normalize(position[i] - position[i + 1]);
			if (velocity[i] == vec3(0.f)) {
				position[i + 1] += bToA  * extraDistance;
			}
			else if (velocity[i + 1] == vec3(0.f)) {
				position[i] += aToB * extraDistance;
			}
			else {
				position[i] += aToB * extraDistance / 2.f;
				position[i + 1] += bToA  * extraDistance / 2.f;
			}	
		}
	}

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
	vec3 meshPosition ( -4 , 9, -4 );
	clothVertexPosition = CreateClothMeshArray(ClothMesh::numRows, ClothMesh::numCols, separation, separation, meshPosition);
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
	if (ImGui::Button("Reset") || timer >= 20) {

		delete[] clothVertexForce;
		delete[] clothVertexPosition;
		delete[] clothVertexPrevPosition;
		delete[] clothVertexVelocity;

		separation = nextSeparation;

		PhysicsInit();

		float HI = 5, LO = -5;
		float r1 = LO + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HI - LO)));
		float r2 = 0 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (5 - 0)));
		float r3 = LO + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (HI - LO)));

		float r4 = 2 + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (4 - 2)));

		spherePosition = { r1, r2, r3};
		sphereRadius = r4;
		timer = 0;
	};

	if (ImGui::Button("Sphere Collision")) {
		sphereCollision = !sphereCollision;
	}
	dt /= 10.f;

for (int j = 0; j < 10; ++j){

	for (int i = 0; i < ClothMesh::numVerts; ++i) {

		if (i != 0 && i != 13) {

			verletSolver(clothVertexPosition[i], clothVertexPrevPosition[i], clothVertexVelocity[i], clothVertexForce[i], 1, dt);

			// Cube collisions
			IsCollidingBox(planeBack, clothVertexPrevPosition[i], clothVertexPosition[i], clothVertexVelocity[i]);
			IsCollidingBox(planeDown, clothVertexPrevPosition[i], clothVertexPosition[i], clothVertexVelocity[i]);
			IsCollidingBox(planeFront, clothVertexPrevPosition[i], clothVertexPosition[i], clothVertexVelocity[i]);
			IsCollidingBox(planeLeft, clothVertexPrevPosition[i], clothVertexPosition[i], clothVertexVelocity[i]);
			IsCollidingBox(planeRight, clothVertexPrevPosition[i], clothVertexPosition[i], clothVertexVelocity[i]);
			IsCollidingBox(planeTop, clothVertexPrevPosition[i], clothVertexPosition[i], clothVertexVelocity[i]);

			//Sphere collision
			if(sphereCollision)	IsCollidingSphere(sphereRadius, spherePosition, clothVertexPosition[i], clothVertexPrevPosition[i], clothVertexVelocity[i]);

			}
	}

	for (int i = 0; i < ClothMesh::numVerts; ++i) {

		springsForceCalc(clothVertexPosition, clothVertexVelocity, clothVertexForce, i, ClothMesh::numRows, ClothMesh::numCols, separation, stretchD, shearD, bendD, stretchK, shearK, bendK);

	}

	for (int i = 0; i < ClothMesh::numVerts; ++i) {

		checkMaxElongation(clothVertexPosition, clothVertexVelocity, i, ClothMesh::numRows, ClothMesh::numCols, separation, elongation);

	}

}
	timer += dt*10;
	ClothMesh::updateClothMesh(&clothVertexPosition[0].x);

}

void PhysicsCleanup() {
	delete[] clothVertexForce;
	delete[] clothVertexPosition;
	delete[] clothVertexPrevPosition;
	delete[] clothVertexVelocity;
}