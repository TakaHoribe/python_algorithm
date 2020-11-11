function validCheck(a0, v0, amin, amax, jmin, jmax) {
    if (a0 < amin || amax < a0) {
      console.log("a0 = ", a0, " amin - ", amin, " amax = ", amax)
      return false
    }
    if (jmax < 0.0 || 0.0 < jmin) {
      return false
    }
    if (v0 < 0.0) {
      return false
    }
    return true
  }
  
  
  function update(t, j, a0, v0, x0) {
    var a = a0 + j * t
    var v = v0 + (a0 * t) + (0.5 * j * t * t)
    var x = x0 + (v0 * t) + (0.5 * a0 * t * t) + (1.0/6.0 * j * t * t * t)
    return [x, v, a]
  }
  
  function calcStopDist(a0, v0, amin, amax, jmin, jmax) {
    var v_end = 0.0;
    var zero_jerk_time = (v_end - v0 + 0.5 * (a0*a0 - amin*amin) / jmin + (0.5 * amin*amin / jmax)) / amin;
    var dec_jerk_time = 0.0;
    var acc_jerk_time = 0.0;
    if (zero_jerk_time > 0) {
      // jerk_plan_type = 'dec_zero_acc'
      dec_jerk_time = (amin - a0) / jmin;
      acc_jerk_time = -amin / jmax;
    } else {
      var min_acc_actual = -Math.sqrt(2.0 * (v_end - v0 + (0.5 * a0**2 / jmin)) * ((jmin * jmax) / (jmax - jmin)));
      if (a0 > min_acc_actual) {
        // jerk_plan_type = 'dec_acc'
        dec_jerk_time = (min_acc_actual - a0) / jmin;
        acc_jerk_time = -min_acc_actual / jmax;
      } else {
        // jerk_plan_type = 'acc'
        dec_jerk_time = 0.0;
        acc_jerk_time = -a0 / jmax;
      }
    }
  
    
    dec_jerk_time = Math.max(dec_jerk_time, 0.0);
    zero_jerk_time = Math.max(zero_jerk_time, 0.0);
    acc_jerk_time = Math.max(acc_jerk_time, 0.0);
  
  
    var state1 = update(dec_jerk_time, jmin, a0, v0, 0.0);
    console.log("x1 = ", state1[0], "v1 = ", state1[1], "a1 = ", state1[2])
    
    var state2 = update(zero_jerk_time, 0.0, state1[2], state1[1], state1[0]);
    console.log("x2 = ", state2[0], "v2 = ", state2[1], "a2 = ", state2[2])
    
    var state3 = update(acc_jerk_time, jmax, state2[2], state2[1], state2[0]);
    console.log("x3 = ", state3[0], "v3 = ", state3[1], "a3 = ", state3[2])
    
    var total_time = dec_jerk_time + zero_jerk_time + acc_jerk_time;
    return [state3[0], state3[1], state3[2], total_time]
  }
  
  
  function myFunction() {
    
    var ss = SpreadsheetApp.getActiveSpreadsheet();
    var sheet = ss.getSheetByName("main");
   
    var max_jerk = sheet.getRange("B2").getValue()
    var min_jerk = sheet.getRange("B3").getValue()
    var max_acc = sheet.getRange("B4").getValue()
    var min_acc = sheet.getRange("B5").getValue()
    var delay_time = sheet.getRange("B6").getValue()
    console.log("max_jerk = ", max_jerk, "min_jerk = ", min_jerk," max_acc = ", max_acc," min_acc = ", min_acc," delay_time = ", delay_time);
  
  
    var range = sheet.getRange("B10:L50");
    var a0arr = sheet.getRange("A10:A50");
    var v0arr = sheet.getRange("B9:BL");
  
    
    for(var row = 1; row <= range.getNumRows(); row++) {
  //  for(var row = 1; row <= 1; row++) {
      var a0 = a0arr.getCell(row, 1).getValue();
      for (var column = 1; column <= range.getNumColumns(); column++) {
        var v0 = v0arr.getCell(1, column).getValue();
        if (validCheck(a0, v0, min_acc, max_acc, min_jerk, max_jerk)) {
          var state = calcStopDist(a0, v0, min_acc, max_acc, min_jerk, max_jerk);
          var x = state[0]
          var v = state[1]
          var a = state[2]
          var t = state[3]
          
          range.getCell(row, column).setValue(x);
        } else {
          range.getCell(row, column).setValue("NaN");
        }
      }
    }
  
  
  }
  