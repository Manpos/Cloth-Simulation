#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>

bool show_test_window = false;
float *clothArray;

namespace ClothMesh {
	extern const int numCols;
	extern const int numRows;
	extern const int numVerts;
	extern void updateClothMesh(float* array_data);
	
}

float* CreateCloth(float separation, int maxRows, int maxColumns) {
	int columns = 0, rows = 0;
	float* result = new float[ClothMesh::numCols * ClothMesh::numRows * 3];
	for (int i = 0; i < (ClothMesh::numCols * ClothMesh::numRows); i++) {
		result[i * 3] = separation * columns;
		result[i * 3 + 2] = separation * rows;
		result[i * 3 + 1] = 0;
		++columns;
		if (columns == maxColumns) {
			columns = 0;
			++rows;
		}
	}
	return result;
}

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

void PhysicsInit() {
	clothArray = CreateCloth(0.5, 18, 14);
}
void PhysicsUpdate(float dt) {
	ClothMesh::updateClothMesh(clothArray);
}
void PhysicsCleanup() {
	//TODO
}