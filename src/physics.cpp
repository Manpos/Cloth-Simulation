#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>

bool show_test_window = false;

namespace ClothMesh {
	extern const int numCols;
	extern const int numRows;
	extern const int numVerts;
	extern void updateClothMesh(float* array_data);
}

float *clothVertexPosition;
float *clothVertexPrevPosition;

float separationX = 1, separationY = 1;

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

float * CreateClothMeshArray(int rowVerts, int columnVerts, float vertexSeparationX, float vertexSeparationY, float position[3]) {
	int currPosX = 0, currPosY = 0;
	float *result = new float[columnVerts * rowVerts * 3];
	for (int i = 0; i < rowVerts * columnVerts; ++i) {
		if ( currPosX < columnVerts) {

			result[i * 3] = position[2] + vertexSeparationY * currPosY;
			result[i * 3 + 1] = position[1] + 0;
			result[i * 3 + 2] = position[0] + vertexSeparationX * currPosX;

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
	float meshPosition[3] = { -4 , 8, -4 };
	clothVertexPosition = CreateClothMeshArray(ClothMesh::numRows, ClothMesh::numCols, 0.5, 0.5, meshPosition);
	clothVertexPrevPosition = new float[ClothMesh::numVerts * 3];



	//TODO
}
void PhysicsUpdate(float dt) {
	//TODO
	ClothMesh::updateClothMesh(clothVertexPosition);
}
void PhysicsCleanup() {
	//TODO
}