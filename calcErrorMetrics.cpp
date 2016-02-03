/*
 * calcErrorMetrics.cpp
 *
 *  Created on: Jan 22, 2016
 *      Author: hrvoje
 */

#include <stdexcept>
#include <iostream>

#include "readGroundTruth.h"
#include "calcErrorMetrics.h"

using namespace std;

void calcErrorMetrics(const char* filename, const vector<flow_t_>& lk_flow, float& angErr, float& magErr)
{
	angErr = 0;
	magErr = 0;
	vector<flow_t_> g_t_flow;
	int not_defined = 0;
	float temp = 0;

	readGroundTruth(filename, lk_flow, g_t_flow);
/*
	cout << "Ground truth flow values: " << endl;
	for (vector<flow_t_>::const_iterator iter = g_t_flow.begin();
			iter != g_t_flow.end(); ++iter) {
		cout << "Row (y) : " << (*iter).pos.y << "  Col (x): " << (*iter).pos.x
				<< endl;
		cout << "Ver. flow (flow_y):  " << (*iter).flow_y << "   Hor. flow:  "
				<< (*iter).flow_x << endl;

	}
	*/


	//cout << g_t_flow.size() << endl;

	if (g_t_flow.size() != lk_flow.size())
		throw domain_error("Flow and ground truth vectors not the same size!");


    //cout << "CHECK!" << endl;
	for (vector<flow_t_>::size_type i = 0; i != lk_flow.size(); ++i) {

		if (( abs(lk_flow[i].flow_x) > 1000) || (abs(lk_flow[i].flow_y) > 1000)
				|| (abs(g_t_flow[i].flow_x) > 1000) || (abs(g_t_flow[i].flow_y) > 1000)) {
			not_defined++;
			//cout << "skipped" << endl;
			continue;
		}

		float u = lk_flow[i].flow_x;
		float v = lk_flow[i].flow_y;
		float u_gt = g_t_flow[i].flow_x;
		float v_gt = g_t_flow[i].flow_y;

		//cout << u << " " << v << " " << u_gt << " " << v_gt << endl;

		temp = acos(
				(1 + u * u_gt + v * v_gt)
						/ (sqrt(1 + u * u + v * v)
								* sqrt(1 + u_gt * u_gt + v_gt * v_gt)));

		if (isnan(temp))
			temp = acos( round ((1 + u * u_gt + v * v_gt)
						/ (sqrt(1 + u * u + v * v)
								* sqrt(1 + u_gt * u_gt + v_gt * v_gt))));

		// temp is introduced after acos value of two "same" numbers divided gave nan value
		// presumably tried to take acos of very small number and fails
		// round prevents that from happening

		angErr = angErr + temp;


		magErr = magErr
				+ sqrt((u - u_gt) * (u - u_gt) + (v - v_gt) * (v - v_gt));



		//cout << angErr << "  " << magErr << endl;
		//cout << endl;


	}

	angErr = angErr / (lk_flow.size() - not_defined);
	magErr = magErr / (lk_flow.size() - not_defined);

}


