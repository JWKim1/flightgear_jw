# flightgear_jw
run flightgear by embedded board



임베디드 보드를 사용하여 비행 시뮬레이션 프로그램인 filght gear를 동작하고 프로그램을 사용하여 보드를 작동시키는 프로그램이 , 
다.

개발환경은 리눅스 운영체제이며, 한 프로세서 안에서 동작해야 한다는 조건에 따라 thread를 생성시켜 동작한다.
보드를 동작시키고 
프로그램에 값을 전송하는 메인 스레드인 client 스레드와 
filght gear에서 정보를 수신하는 server 스레드가 실행되면서 프로그램이 동작한다.

server 스레드에서는 flight gear 로부터 지정된 시간마다 주기적으로 비행정보를 입력받아 변수에 값을 저장한다. 
client 스레드는 server 스레드가 저장한 값을 바탕으로 
보드의 각 부품을 수행시키기 위한 스레드를 생성하고, 
보드의 상태에 따라 비행기를 조정하기 위해 
조작에 필요한 변수를 filght gear 에게 전송한다. 
기타 스레드는 보드 의 한 device를 동작하기 위한 스레드들이다.

단순히 비행상태를 보드에 나타내고 
보드의 스위치를 사용해 프로그램을 단순조작하는것에 그치지 않고
초기 상태에서 지정한 고도까지 자동 이륙하는 auto start와 
일정 고도에서 자율주행이 가능하게 만드는 등 고급 비행기능을 추가하였다.