Standing-Egg Sensor EVK 프로그램 이다.<br>
Atmel Studio tool이 사용 되었으며 <B>(주)스탠딩 에그</B>에서 제작한 SGA100 센서를 읽는 코드와 
Atmel cdc driver를 통해 wifi 및 pc와 serial 통신을 하는 프로젝트 이다.

코드 내용은 "./sensor_eval_wifi/src/main.c" 올라와져 있다.

SGA100 address는 <B>0x64</B>
packet 전송은 <B>PKST</B> 로 시작하며 뒤에 hex code로 x,y,z axis 값이 보내진다.


