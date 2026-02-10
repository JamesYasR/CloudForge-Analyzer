#include "Basic/ColorManager.h"

ColorManager::ColorManager(double R, double G, double B) {
	r = R;
	g = G;
	b = B;
}
ColorManager::ColorManager() {
	r = rand() % 256;
	g = rand() % 256;
	b = rand() % 256;
}
ColorManager:: ~ColorManager() {
	
}
