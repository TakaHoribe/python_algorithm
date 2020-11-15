function validCheck(a0, v0, amin, amax, jmin, jmax) {
  if (a0 < amin || amax < a0) {
//    console.log("a0 = ", a0, " amin - ", amin, " amax = ", amax)
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

function calcStopDist(a0, v0, amin, amax, jmin, jmax, delay_time) {

  var x0 = 0.0;
  //  console.log("x_ini = ", 0.0, "v_init = ", v0, "a_ini = ", a0)
  var state0 = update(delay_time, 0.0, a0, v0, x0);
  x0 = state0[0]
  v0 = state0[1]
  a0 = state0[2]
  //  console.log("x0 = ", state0[0], "v0 = ", state0[1], "a0 = ", state0[2])


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


  var state1 = update(dec_jerk_time, jmin, state0[2], state0[1], state0[0]);
  //  console.log("x1 = ", state1[0], "v1 = ", state1[1], "a1 = ", state1[2])

  var state2 = update(zero_jerk_time, 0.0, state1[2], state1[1], state1[0]);
  //  console.log("x2 = ", state2[0], "v2 = ", state2[1], "a2 = ", state2[2])

  var state3 = update(acc_jerk_time, jmax, state2[2], state2[1], state2[0]);
  //  console.log("x3 = ", state3[0], "v3 = ", state3[1], "a3 = ", state3[2])

  var total_time = dec_jerk_time + zero_jerk_time + acc_jerk_time + delay_time;
  return [state3[0], state3[1], state3[2], total_time]
}


function calculate() {

  var sheet = SpreadsheetApp.getActiveSpreadsheet().getSheetByName("stop_distance");


  var value_range_str = "F4:AJ54";
  var a0_range_str = "E4:E54";
  var v0_range_str = "F3:AJ3";
  var parameter_range_str = "B4:B8";

  // get parameters
  var parameters = sheet.getRange(parameter_range_str).getValues();
  var max_jerk = parameters[0][0];
  var min_jerk = parameters[1][0];
  var max_acc = parameters[2][0];
  var min_acc = parameters[3][0];
  var delay_time = parameters[4][0];

  // get range
  var range = sheet.getRange(value_range_str);
  var range_time = SpreadsheetApp.getActiveSpreadsheet().getSheetByName("stop_time").getRange(value_range_str);
  var a0arr = sheet.getRange(a0_range_str).getValues();
  var v0arr = sheet.getRange(v0_range_str).getValues();


  var ROW_NUM = range.getNumRows();
  var COL_NUM = range.getNumColumns();

  // initialize table
  var dist_table = new Array(ROW_NUM);
  for(let i = 0; i < dist_table.length; i++) {
    dist_table[i] = new Array(COL_NUM).fill("NaN");
  }
  var time_table = new Array(ROW_NUM);
  for(let i = 0; i < time_table.length; i++) {
    time_table[i] = new Array(COL_NUM).fill("NaN");
  }

  // calculate values
  for(var row = 0; row < ROW_NUM; row++) {
    var a0 = a0arr[row][0];
    for (var column = 0; column < COL_NUM; column++) {
      var v0 = v0arr[0][column];
      if (validCheck(a0, v0, min_acc, max_acc, min_jerk, max_jerk)) {
        var state = calcStopDist(a0, v0, min_acc, max_acc, min_jerk, max_jerk, delay_time);
        var x = state[0]
        var v = state[1]
        var a = state[2]
        var t = state[3]

        if (Math.abs(a) < 0.1 && Math.abs(v) < 0.1) {
          dist_table[row][column] = x;
          time_table[row][column] = t;
        } else {
          // ジャーク制約を守って停止できない（終端条件a=0が守れない）場合の計算をする（終端はa=0でなくても良い場合。）
        }
      }
    }
  }

  // write values on sheet
  range.setValues(dist_table);
  range_time.setValues(time_table);

}
