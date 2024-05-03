#include "Lane_Change.h"

void Lane_Change::TurnonLaneChangersSignal() {
	Lanechanger = std::vector<LaneChangerInformation>(0);
	MeasuredThisTime.Lanechange_original = 0;
	MeasuredThisTime.Lanechange_Pushed = 0;
	LaneChangerInformation changer;
	BothLaneChangerInformation both;
	for (int i = 0; i < car.List_Defector.size(); ++i) {
		changer.signal = Car_information::SignalKind::Non;
		int ID = car.List_Defector[i];
		InsentiveInformation right, left;
		left = _CheckInsentives(ID, Car_information::SignalKind::Left);
		right = _CheckInsentives(ID, Car_information::SignalKind::Right);
		if (left.on == false && right.on == false) {
			if (car.strategy[ID] == Car_information::StrategyKind::D && car.headway.current[ID] <= car.velocity.current[ID] - car.velocity.current[car.around.preceding.current[ID]]) {
				WillbePusher.emplace(ID);
				continue;
			}
			else if(random->random(1.0)<car.velocity.current[ID]*0.01&&car.velocity.current[ID]!=0) {
			    if (random->random(1.0) < 0.5)  changer.signal = Car_information::SignalKind::Left;
			    else changer.signal = Car_information::SignalKind::Right;
			}
			else continue;
		}
		if (left.on == true || right.on == true) {
			if (left.distance > right.distance) changer.signal = Car_information::SignalKind::Left;
			else if (left.distance < right.distance) changer.signal = Car_information::SignalKind::Right;
			else {
				if (random->random(1.0) < 0.5)  if (left.on) changer.signal = Car_information::SignalKind::Left;
				else if (right.on) changer.signal = Car_information::SignalKind::Right;
			}
			if (changer.signal != Car_information::SignalKind::Non) {
				changer.ID = ID;
				changer.position = car.position.current[ID];
				int v = car.lanenumber.current[changer.ID];
				Lanechanger.emplace_back(changer);
				both.info = changer;
				both.isPushed = false;
				TotalLaneChanger.emplace_back(both);
				AlreadyChosen.emplace(ID);
				if (CanditateLeadingCar[car.lanenumber.current[changer.ID]].distance < car.headway.current[changer.ID]) {
					CanditateLeadingCar[car.lanenumber.current[changer.ID]].ID = changer.ID;
					CanditateLeadingCar[car.lanenumber.current[changer.ID]].distance = car.headway.current[changer.ID];
				}
			}
		}
	}
}


void Lane_Change::PickUpPushed() {
	PushedVehicleInformation PushedVehicle;
	BothLaneChangerInformation both;
	int BackwardSafetyDistance = 3;
	for (int i = 0; i < constants.N; ++i) {
		if (AlreadyChosen.count(i) != 0) continue;
		int followerID = car.around.following.current[i];
		int distance = car.headway.current[followerID];
		bool isFocalVehiclePushed = false;
		int VelocityDefference = car.velocity.current[i] - car.velocity.current[followerID];
		//		if (VelocityDefference < 0 && WillbePusher.count(followerID) == 1) isFocalVehiclePushed = true;
	        if (VelocityDefference < 0&&car.strategy[followerID]==Car_information::StrategyKind::D) isFocalVehiclePushed = true;
		//else if (VelocityDefference < 0 && car.strategy[followerID] == Car_information::StrategyKind::C) car.canditate_velocity[followerID] = std::max(1, car.canditate_velocity[followerID] - 1);
		//If Following Vehicle which wants to "Push" is approaching focal vehicle and follower is faster than focal, focal vehicle feel "Pushed"
		if (distance <= BackwardSafetyDistance && isFocalVehiclePushed) {
			PushedVehicle.info.ID = i;
			PushedVehicle.info.position = car.position.current[i];
			PushedVehicle.info.signal = Car_information::SignalKind::Non;
			int FocalLane = car.lanenumber.current[i];
			CanditateAroundVehicle right, left;
			if (0 < FocalLane && FocalLane < constants.Numberoflane - 1) {
				int NextLane = FocalLane + 1;
				right = _GetAroundInformation(i, NextLane);
				NextLane = FocalLane - 1;
				left = _GetAroundInformation(i, NextLane);
				if(right.following.distance==0&&left.following.distance==0) continue;
				if (right.following.distance > left.following.distance) PushedVehicle.info.signal = Car_information::SignalKind::Right;
				else if (right.following.distance < left.following.distance) PushedVehicle.info.signal = Car_information::SignalKind::Left;
				else {
				        if (random->random(1.0) < 0.5) PushedVehicle.info.signal = Car_information::SignalKind::Left;
					else PushedVehicle.info.signal = Car_information::SignalKind::Right;
				}
			}
			else if (FocalLane == 0) {
				int NextLane = FocalLane + 1;
				right = _GetAroundInformation(i, NextLane);
				PushedVehicle.info.signal = Car_information::SignalKind::Right;
			}
			else {
				int NextLane = FocalLane - 1;
				left = _GetAroundInformation(i, NextLane);
				PushedVehicle.info.signal = Car_information::SignalKind::Left;
			}
			if (PushedVehicle.info.signal != Car_information::SignalKind::Non) {
				if (PushedVehicle.info.signal == Car_information::SignalKind::Left) PushedVehicle.around = left;
				if (PushedVehicle.info.signal == Car_information::SignalKind::Right) PushedVehicle.around = right;
				both.info = PushedVehicle.info;
				both.isPushed = true;
				TotalLaneChanger.emplace_back(both);
				car.pushing.isPushing[car.around.following.current[i]] = true;
				car.pushing.Preceding[car.around.following.current[i]] = i;
				if (CanditateLeadingCar[car.lanenumber.current[both.info.ID]].distance < car.headway.current[both.info.ID]) {
					CanditateLeadingCar[car.lanenumber.current[both.info.ID]].ID = both.info.ID;
					CanditateLeadingCar[car.lanenumber.current[both.info.ID]].distance = car.headway.current[both.info.ID];
				}
			}
		}
	}
}


bool Lane_Change::TryLaneChange() {
	bool isLaneChangeDone = false;
	MeasuredThisTime.Lanechange_original = 0;
    MeasuredThisTime.Lanechange_Pushed = 0;
	if (TotalLaneChanger.size() > 0) {
		TotalLaneChanger = _DecideUpdateOrder();
		BothLaneChangerInformation LI;
		CanditateAroundVehicle around;
		for (int i = 0; i < TotalLaneChanger.size(); ++i) {
			LI = TotalLaneChanger[i];
			LI.info.position = car.position.current[LI.info.ID];
			int FocalLane = car.lanenumber.current[LI.info.ID];
			int NextLane = FocalLane + 1;
			if (LI.info.signal == Car_information::SignalKind::Left) NextLane -= 2;
			if (map.recorded.existence.current[NextLane][LI.info.position]) continue;
			around = _GetAroundInformation(LI.info.ID, NextLane);
			bool beforeLaneChange = true;
			if (LI.isPushed && around.preceding.distance < car.velocity.current[LI.info.ID] - car.velocity.current[around.preceding.ID]) continue;
			if (car.pushing.isPushing[LI.info.ID]){
				if (car.lanenumber.current[car.pushing.Preceding[LI.info.ID]] != car.lanenumber.previous[car.pushing.Preceding[LI.info.ID]]) continue;
			}
			if (around.following.distance >= car.velocity.current[around.following.ID] - car.velocity.current[LI.info.ID]) {
				isLaneChangeDone = true;
				if (!LI.isPushed) ++MeasuredThisTime.Lanechange_original;
				else ++MeasuredThisTime.Lanechange_Pushed;
				--map.eachlanevehicle[FocalLane];
				map.recorded.existence.current[FocalLane][LI.info.position] = false;
				if (map.eachlanevehicle[FocalLane] > 0) _UpdateRelationship(LI.info, around, beforeLaneChange);
				else car.leadingvehicle[FocalLane].existence = false;
				beforeLaneChange = false;
				car.lanenumber.current[LI.info.ID] = NextLane;
				++map.eachlanevehicle[NextLane];
				map.recorded.existence.current[NextLane][LI.info.position] = true;
				map.recorded.ID.current[NextLane][LI.info.position] = LI.info.ID;
				_UpdateRelationship(LI.info, around, beforeLaneChange);
			}
		}
		for (int Lane = 0; Lane < constants.Numberoflane; ++Lane) {
			if (map.eachlanevehicle[Lane] == 0) car.leadingvehicle[Lane].existence = false;
			else {
				Car_information::LeadingVehicle Leading;
				Leading = car.leadingvehicle[Lane];
				int ID = car.around.following.current[Leading.ID];
				while (ID != Leading.ID) {
					if (Leading.distance < car.headway.current[ID]) {
						Leading.distance = car.headway.current[ID];
						Leading.ID = ID;
					}
					ID = car.around.following.current[ID];
				}
			}
		}
		return isLaneChangeDone;
	}
	else return isLaneChangeDone;
}


Lane_Change::InsentiveInformation Lane_Change::_CheckInsentives(int ID, int signal) {
	InsentiveInformation check;
	CanditateAroundVehicle Around;
	check.distance = 0;
	check.on = false;
	int FocalLane = car.lanenumber.current[ID];
	int precedingcarID = car.around.preceding.current[ID];
	if (FocalLane == 0 && signal == Car_information::SignalKind::Left) return check;
	else if (FocalLane == constants.Numberoflane - 1 && signal == Car_information::SignalKind::Right) return check;
	else {
		FocalLane++;
		if (signal == Car_information::SignalKind::Left) FocalLane -= 2;
		if (map.recorded.existence.current[FocalLane][car.position.current[ID]]) return check;
		if (car.headway.current[ID] > car.velocity.current[ID] - car.velocity.current[precedingcarID]) return check;
		Around = _GetAroundInformation(ID, FocalLane);
		if (Around.preceding.distance > car.velocity.current[ID] - car.velocity.current[Around.preceding.ID] && Around.preceding.distance > 0) {
			check.on = true;
			check.distance = Around.preceding.distance;
		}
	}
	return check;
}

Lane_Change::CanditateAroundVehicle Lane_Change::_GetAroundInformation(int ID, int FocalLane) {
	CanditateAroundVehicle around;
	if (map.eachlanevehicle[FocalLane] > 0) {
		int PrecedingPosition = car.position.current[ID];
		int FollowingPosition = car.position.current[ID] - 1;
		while (true) {
			if (PrecedingPosition >= constants.lanelength) PrecedingPosition -= constants.lanelength;
			if (FollowingPosition < 0) FollowingPosition += constants.lanelength;
			bool isPrecedingisDetected = map.recorded.existence.current[FocalLane][PrecedingPosition];
			bool isFollowingisDetected = map.recorded.existence.current[FocalLane][FollowingPosition];
			if (isPrecedingisDetected or isFollowingisDetected) {
				if (isPrecedingisDetected) {
					around.preceding.ID = map.recorded.ID.current[FocalLane][PrecedingPosition];
					around.preceding.distance = PrecedingPosition - car.position.current[ID] - 1;
					if (around.preceding.distance < 0) around.preceding.distance += constants.lanelength;
					around.following.ID = car.around.following.current[around.preceding.ID];
					around.following.distance = car.position.current[ID] - car.position.current[around.following.ID] - 1;
					if (around.following.distance < 0) around.following.distance += constants.lanelength;
					break;
				}
				else {
					around.following.ID = map.recorded.ID.current[FocalLane][FollowingPosition];
					around.following.distance = car.position.current[ID] - FollowingPosition - 1;
					if (around.following.distance < 0) around.following.distance += constants.lanelength;
					around.preceding.ID = car.around.preceding.current[around.following.ID];
					around.preceding.distance = car.position.current[around.preceding.ID] - car.position.current[ID] - 1;
					if (around.preceding.distance < 0) around.preceding.distance += constants.lanelength;
					break;
				}
			}
			else {
				++PrecedingPosition;
				--FollowingPosition;
			}
		}
	}
	else {
		around.following.distance = constants.lanelength - 1;
		around.preceding.distance = constants.lanelength - 1;
		around.preceding.ID = around.following.ID = ID;
	}
	return around;
}

std::vector<Lane_Change::BothLaneChangerInformation> Lane_Change::_DecideUpdateOrder() {
	std::vector<BothLaneChangerInformation> Order;
	std::vector<CanditateAroundVehicle::Detected> TrueCanditateLeadingVehicle;
	for (int i = 0; i < constants.Numberoflane; ++i) if (CanditateLeadingCar[i].ID != -1) TrueCanditateLeadingVehicle.emplace_back(CanditateLeadingCar[i]);
	int picker;
	int leaderID;
	if (TrueCanditateLeadingVehicle.empty()) {
		picker = 0;
		leaderID = TotalLaneChanger[picker].info.ID;
	}
	else {
		picker = random->random((int)TrueCanditateLeadingVehicle.size() - 1);
		leaderID = TrueCanditateLeadingVehicle[picker].ID;
	}
	int StartingPosition = car.position.current[leaderID];
	BothLaneChangerInformation changer;
	for (int i = 0; i < TotalLaneChanger.size(); ++i) {
		changer = TotalLaneChanger[i];
		changer.info.position = car.position.current[changer.info.ID];
		if (changer.info.position <= StartingPosition) changer.info.position += constants.lanelength - StartingPosition - 1;
		else changer.info.position -= StartingPosition + 1;
		Order.emplace_back(changer);
	}
	std::sort(Order.begin(), Order.end(), [](BothLaneChangerInformation const& fr, BothLaneChangerInformation const& se) {
		return fr.info.position > se.info.position;
		});
	return Order;
}


void Lane_Change::_UpdateRelationship(LaneChangerInformation LI, CanditateAroundVehicle around, bool beforeLaneChange) {
	int FocalLane = car.lanenumber.current[LI.ID];
	if (beforeLaneChange) {
		int Preceding = car.around.preceding.current[LI.ID];
		int Following = car.around.following.current[LI.ID];
		int newHeadway = car.position.current[Preceding] - car.position.current[Following] - 1;
		if (newHeadway < 0) newHeadway += constants.lanelength;
		car.around.following.current[Preceding] = Following;
		car.around.preceding.current[Following] = Preceding;
		car.headway.current[Following] = newHeadway;
		if (LI.ID == car.leadingvehicle[FocalLane].ID)  car.leadingvehicle[FocalLane].ID = Following;
	}
	else {
		car.around.following.current[LI.ID] = around.following.ID;
		car.around.preceding.current[LI.ID] = around.preceding.ID;
		car.around.preceding.current[around.following.ID] = car.around.following.current[around.preceding.ID] = LI.ID;
		car.headway.current[LI.ID] = around.preceding.distance;
		car.headway.current[around.following.ID] = around.following.distance;
	}
	if (map.eachlanevehicle[FocalLane] == 1) {
		car.around.following.current[LI.ID] = car.around.preceding.current[LI.ID] = LI.ID;
		car.headway.current[LI.ID] = constants.lanelength - 1;
		car.leadingvehicle[FocalLane].existence = true;
		car.leadingvehicle[FocalLane].ID = LI.ID;
	}
}

Lane_Change::LaneChangerInformation::LaneChangerInformation() {
	this->ID = -1;
	this->position = -1;
	this->signal = Car_information::SignalKind::Non;
}

void Lane_Change::InitializeLaneChangerInfomation() {
	TotalLaneChanger = std::vector<BothLaneChangerInformation>(0);
	car.pushing.isPushing = std::vector<bool>(constants.N, false);
	car.pushing.Preceding = std::vector<int>(constants.N, -1);
	Pushed = std::vector<PushedVehicleInformation>(0);
	WillbePusher.clear();
	AlreadyChosen.clear();
	CanditateLeadingCar = std::vector<CanditateAroundVehicle::Detected>(constants.Numberoflane);
	car.pushing.Preceding.assign(constants.N, -1);
	car.pushing.isPushing.assign(constants.N, false);
	for (int i = 0; i < constants.Numberoflane; ++i) {
		CanditateLeadingCar[i].ID = -1;
		CanditateLeadingCar[i].distance = -1;
	}
}

