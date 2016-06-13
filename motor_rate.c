

switch(value){
	case: 5
		turn_on();
		value--;
		break;
	case: 4
		turn_on();
		value--;
		break;
	case: ...

	case: 1
		turn_off();
		value = read_value();
		break;
}



switch(fan_speed_buf[i]){
	case: 5 
		turn_on();
		fan_speed_buf[i]--;
		break;
	case: 4 //80%
		turn_on();
		fan_speed_buf[i]--;
		break;
	case: 3 //75%
		turn_on();
		fan_speed_buf[i]--;
		break;
	case: 2 //67%
		turn_on();
		fan_speed_buf[i]--;
		break;
	case: 1 //50%
		turn_on();
		fan_speed_buf[i]--;
		break;
	case: 0 //reset buf when it reach to 0(zero)
		turn_off();
		fan_speed_buf[i] = fan_state_old[i];
		break;
}

80% oooox
75% ooox
67% oox
50% ox
// 33% oxx
// 25% oxxx

//=============================//
if(fan_speed_buf[i]>0){
	turn_on();
	fan_speed_buf[i]--;
}
else{
	turn_off();
	fan_speed_buf[i] = fan_state_old[i];
}