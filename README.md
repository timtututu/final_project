再將code燒錄進入板子之後，就可以把車至於黑線的起點上，接著車子會在第一個branck做左轉的動作，如果遇到障礙物會做一個迴轉
並且迴轉回回到前一個branck重新選擇一條新的路徑，如過在地圖上偵測到3個以上的remark點同時又偵測到障礙物就會觸發慶祝模式(原地高速打轉)
將erpc連線上並執行car_control.py檔，按下任一個方向鍵都可以得到目前車子的行駛狀態(包含速度、行駛距離......各種狀態)，按下q可以退出。

After copying the code into the board, you can put the car at the starting point of the black line. Then the car will turn left at the first branch. If it encounters an obstacle, it will make a turn and return to the previous branch. Re-select a new path. If more than 3 mark points are detected on the map and obstacles are detected simultaneously, the celebration mode will be triggered (spin in place at high speed). Connect the eRPC (Embedded Remote Procedure Call) online and execute the car_control.py file. , press any direction key to get the current driving status of the car (including speed, driving distance..., and various statuses), and press q to exit.
