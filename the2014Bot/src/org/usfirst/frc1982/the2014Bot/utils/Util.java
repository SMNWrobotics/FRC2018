package org.usfirst.frc1982.the2014Bot.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class Util {
	public GameScenario AutoSelector(int pos) {
    	String GameData = DriverStation.getInstance().getGameSpecificMessage();
    	char target = GameData.charAt(0);
    	if (target == 'L' | target == 'R') {
    		// starting on left
    		if (pos == 1) {
    			if (target == 'L') {
    				return GameScenario.NEAR;
    			}else {
    				return GameScenario.FAR;
    			}
    	
	    	//starting in the middle
	    	}else if (pos == 2) {
	    		if (target == 'L') {
	    			return GameScenario.INNER_LEFT;
	    		}else {
	    			return GameScenario.INNER_RIGHT;
	    		}
	    	
	    	//starting on the right
	    	}else if (pos == 3) {
	    		if (target == 'R') {
	    			return GameScenario.NEAR;
	    		}else {
	    			return GameScenario.FAR;
	    		}
    	
	    	//position entered wrong
	    	}else {
	    		System.out.println("position wrong");
	    		return GameScenario.DO_NOTHING;
	    	}
    	}else {
    		//game data is wrong
    		System.out.println("Game data wrong");
    		System.out.println(GameData);
    		System.out.println(target);
    		return GameScenario.DO_NOTHING;
    	}
    	
    }
}
