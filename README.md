 Contral API - Robot Control via MindServer

MindServer는 모든 로봇 제어를 중앙 집중화합니다. Contral (Minecraft Mod)은 HTTP REST API를 통해 로봇을 제어할 수 있습니다.

## 기본 정보

- **서버**: `http://localhost:8080` (MindServer와 동일한 PC)
- **포트 충돌 없음**: 기존 MindServer 8080 포트에 `/robot/*` 엔드포인트 추가

## Lock 정책

### Lock이 필요한 명령 (Motion)
- `waveHand`, `applaud`, `jump` 등 모든 모션 명령
- 외부 RL이 락을 잡고 있으면 Agent/Contral은 모션 실행 불가

### Lock이 필요 없는 명령
- `blink` (TTS 눈 깜빡임)
- `track` (트래킹)
- `camera` (카메라 캡처)

### Fail-Open 정책
- MindServer에 접속 불가시 → 명령 허용 (안전 우선)

---

## REST API Endpoints

### 1. Lock 상태 확인

```bash
# Lock 상태 조회
GET /robot/lock

# Response:
{
  "isLocked": false,
  "owner": null,
  "ownerType": null,
  "acquiredAt": null,
  "taskId": null,
  "taskType": null,
  "durationMs": 0
}
```

### 2. Lock 획득 (Contral용)

```bash
# Contral이 Lock 획득
POST /robot/lock/acquire
Content-Type: application/json

{
  "requesterId": "contral",
  "requesterType": "contral",
  "force": false
}

# Response:
{
  "success": true,
  "lock": {
    "isLocked": true,
    "owner": "contral",
    "ownerType": "contral",
    ...
  }
}
```

### 3. Lock 해제

```bash
# Lock 해제
POST /robot/lock/release
Content-Type: application/json

{
  "requesterId": "contral",
  "requesterType": "contral"
}
```

### 4. 실행 가능 여부 확인

```bash
# Agent가 모션 실행 가능한지 확인
GET /robot/can-execute?agent=AgentName

# Response:
{
  "canExecute": true,
  "lock": { ... }
}
```

### 5. 모션 실행

```bash
# 특정 모션 ID 실행
POST /robot/motion
Content-Type: application/json

{
  "motionId": 38,
  "agentName": "contral"
}

# 또는 이름으로 실행
POST /robot/waveHand
POST /robot/jump
POST /robot/applaud
...
```

### 6. Blink 제어 (Lock 불필요)

```bash
POST /robot/blink
Content-Type: application/json

{ "state": "on" }   # 또는 "off", "toggle"
```

### 7. Health Check

```bash
GET /robot/health

# Response:
{
  "online": true,
  "latency": 45,
  "status": {
    "blinkMode": false,
    "trackMode": true
  },
  "connection": { ... },
  "lock": { ... }
}
```

---

## Contral Java 예제 코드

```java
// ContralRobotClient.java

import java.net.HttpURLConnection;
import java.net.URL;
import java.io.*;

public class ContralRobotClient {
    private static final String MIND_SERVER = "http://localhost:8080";
    
    // Lock 상태 확인
    public static boolean isLocked() {
        try {
            String response = get("/robot/lock");
            return response.contains("\"isLocked\":true");
        } catch (Exception e) {
            return false; // Fail-open: 접속 불가시 잠금 없음으로 처리
        }
    }
    
    // Lock 획득
    public static boolean acquireLock() {
        try {
            String body = "{\"requesterId\":\"contral\",\"requesterType\":\"contral\"}";
            String response = post("/robot/lock/acquire", body);
            return response.contains("\"success\":true");
        } catch (Exception e) {
            return true; // Fail-open
        }
    }
    
    // Lock 해제
    public static void releaseLock() {
        try {
            String body = "{\"requesterId\":\"contral\",\"requesterType\":\"contral\"}";
            post("/robot/lock/release", body);
        } catch (Exception e) {
            // Ignore
        }
    }
    
    // 모션 실행 (Lock 확인 후)
    public static boolean executeMotion(String motionName) {
        try {
            if (isLocked()) {
                System.out.println("Robot is locked, cannot execute: " + motionName);
                return false;
            }
            
            String response = post("/robot/" + motionName, "{}");
            return response.contains("\"success\":true");
        } catch (Exception e) {
            return false;
        }
    }
    
    // HTTP GET
    private static String get(String path) throws Exception {
        URL url = new URL(MIND_SERVER + path);
        HttpURLConnection conn = (HttpURLConnection) url.openConnection();
        conn.setRequestMethod("GET");
        conn.setConnectTimeout(500);
        conn.setReadTimeout(500);
        
        BufferedReader reader = new BufferedReader(
            new InputStreamReader(conn.getInputStream())
        );
        StringBuilder sb = new StringBuilder();
        String line;
        while ((line = reader.readLine()) != null) {
            sb.append(line);
        }
        reader.close();
        return sb.toString();
    }
    
    // HTTP POST
    private static String post(String path, String body) throws Exception {
        URL url = new URL(MIND_SERVER + path);
        HttpURLConnection conn = (HttpURLConnection) url.openConnection();
        conn.setRequestMethod("POST");
        conn.setRequestProperty("Content-Type", "application/json");
        conn.setDoOutput(true);
        conn.setConnectTimeout(500);
        conn.setReadTimeout(500);
        
        OutputStream os = conn.getOutputStream();
        os.write(body.getBytes());
        os.close();
        
        BufferedReader reader = new BufferedReader(
            new InputStreamReader(conn.getInputStream())
        );
        StringBuilder sb = new StringBuilder();
        String line;
        while ((line = reader.readLine()) != null) {
            sb.append(line);
        }
        reader.close();
        return sb.toString();
    }
}
```

### 사용 예시

```java
// WASD 테스트 시작 전
if (ContralRobotClient.acquireLock()) {
    // 테스트 진행
    ContralRobotClient.executeMotion("waveHand");
    ContralRobotClient.executeMotion("jump");
    
    // 테스트 완료 후 Lock 해제
    ContralRobotClient.releaseLock();
}

// 또는 Lock 없이 실행 (다른 곳에서 사용 중이면 실패)
ContralRobotClient.executeMotion("applaud");
```

---

## 아키텍처 요약

```
┌─────────────────────────────────────────────────────────────┐
│                        MindServer (8080)                     │
│  ┌─────────────────────────────────────────────────────┐    │
│  │              RobotService (Singleton)                │    │
│  │  ┌───────────┐ ┌──────────────┐ ┌───────────────┐   │    │
│  │  │   Lock    │ │  Motion API  │ │  Blink/Track  │   │    │
│  │  │  Manager  │ │ (lock check) │ │  (no lock)    │   │    │
│  │  └───────────┘ └──────────────┘ └───────────────┘   │    │
│  └─────────────────────────────────────────────────────┘    │
│                           │                                  │
│  ┌────────────────────────┼────────────────────────────┐    │
│  │        HTTP API        │        Socket.IO           │    │
│  │    /robot/* endpoints  │    robot-* events          │    │
│  └────────────────────────┼────────────────────────────┘    │
└───────────────────────────┼─────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
        ▼                   ▼                   ▼
   ┌─────────┐        ┌──────────┐       ┌──────────────┐
   │ Contral │        │  Agent   │       │ External RL  │
   │  (Java) │        │  (JS)    │       │    (Python)  │
   │  HTTP   │        │ Socket.IO│       │    HTTP      │
   └─────────┘        └──────────┘       └──────────────┘
        │                   │                   │
        └───────────────────┴───────────────────┘
                            │
                            ▼
                    ┌──────────────┐
                    │ C++ Robot    │
                    │ Server :8080 │
                    │ (외부 IP)     │
                    └──────────────┘
```

---

## 주의사항

1. **포트 구분**: MindServer는 `localhost:8080`, C++ Robot Server는 외부 IP `121.174.4.243:8080`
2. **Fail-Open**: MindServer 접속 불가시 명령 허용 (로봇 안전 우선)
3. **Stale Lock 자동 해제**: External RL Lock이 5분 이상 유지되면 자동 해제
4. **TTS Blink**: Lock 상관없이 항상 동작 (대화 중 눈 깜빡임 필수)
