#include <TL-Engine.h>
#include <vector>
#include <cmath>

using namespace tle;
using namespace std;

// --- Constants ---
const float kPI = 3.1415926535f;
const float kMaxSpeedMph = 350.0f;
const float kMphToMps = 0.44704f;
const float kMaxSpeed = kMaxSpeedMph * kMphToMps;
const float kMaxReverseSpeed = 40.0f * kMphToMps;
const float kThrustForce = 700.0f;
const float kReverseThrustForce = 200.0f;
const float kDragCoefficient = 0.02f;
const float kTurnSpeed = 180.0f;
const float kHealthDecrease = 1.0f;
const float kCameraSpeed = 50.0f;
const float kCameraInitialX = 0.0f;
const float kCameraInitialY = 10.0f;
const float kCameraInitialZ = -30.0f;
const float kCarRadius = 2.0f;
const float kBarrierRadius = 2.0f;
const float kTankRadius = 2.0f;
const float kCarScale = 0.7f;
const float kBarrierScale = 1.0f;
const float kTankScale = 0.8f;
const float kMinSpeedThreshold = 0.01f;
const float kMomentumDamping = -0.5f;
const float kPushDistance = 1.0f;
const float kInitialHealth = 100.0f;
const float kCountdownDuration = 3.0f;
const float kMouseSensitivity = 50.0f;
const float kMaxPitchAngle = 60.0f;
const float kFirstPersonY = 5.0f;
const float kFirstPersonZ = 2.0f;
const int kGameStateTextX = 20.0f;
const int kGameStateTextOffsetY = 90.0f;
const int kStatsTextOffsetY = 50.0f;
const int kHealthTextX = 300.0f;
const float kLowHealthThreshold = 20.0f;
const float kBarrierGap = 2.0f; // Small gap for visual clearance

// Track
const int kNumTrackPoints = 64;
const float kRoadRadius = 180.0f;
const float kRoadWidth = 28.0f;
const int kNumCheckpoints = 4;
const int kBarriersPerSegment = 10; // High density for no space
const int kBarriersPerSegmentStartArch = 3; // Fewer barriers at start arch

// Laps
const int kTotalLaps = 2;

// Game States
enum GameState
{
    START_MENU,
    COUNTDOWN,
    RACING,
    FINISHED
};

enum CameraView
{
    CHASE_CAM,
    FIRST_PERSON,
    BIRD_VIEW
};

struct Vector2D
{
    float x, z;
    Vector2D operator+ (const Vector2D& other) const { return { x + other.x, z + other.z }; }
    Vector2D operator* (float scalar) const { return { x * scalar, z * scalar }; }
};

struct BoundingBox
{
    float leftX, backZ, rightX, frontZ;
    bool IsPointInside(float x, float z) const
    {
        return (x >= leftX && x <= rightX) && (z >= backZ && z <= frontZ);
    }
};

struct Checkpoint
{
    IModel* model;
    bool passed;
    BoundingBox bounds;
};

void ApplyScale(IModel* model, float scale) { model->Scale(scale); }
void ApplyRotation(IModel* model, float xAngle, float yAngle) { model->RotateX(xAngle); model->RotateY(yAngle); }

Vector2D CalculateThrustVector(IModel* car, float thrust)
{
    float angle = -car->GetY() * kPI / 180.0f;
    return { sin(angle) * thrust, cos(angle) * thrust };
}

Vector2D CalculateDragVector(const Vector2D& momentum, float frameTime)
{
    float speed = sqrt(momentum.x * momentum.x + momentum.z * momentum.z);
    if (speed > 0.0f)
    {
        float dragForce = kDragCoefficient * speed * speed * frameTime;
        return { momentum.x * (-dragForce / speed), momentum.z * (-dragForce / speed) };
    }
    return { 0.0f, 0.0f };
}

bool CheckPointCollision(IModel* car, Checkpoint& checkpoint)
{
    float carX = car->GetX();
    float carZ = car->GetZ();
    return checkpoint.bounds.IsPointInside(carX, carZ);
}

bool CircleCollision2D(float object1X, float object1Z, float radius1,
    float object2X, float object2Z, float radius2)
{
    float dx = object1X - object2X;
    float dz = object1Z - object2Z;
    float distanceSquared = dx * dx + dz * dz;
    float radiusSum = radius1 + radius2;
    return distanceSquared < (radiusSum * radiusSum);
}

void HandleCarCollision(IModel* car, float objectX, float objectZ, float objectRadius,
    float& carHealth, Vector2D& momentum, float pushStrength)
{
    if (CircleCollision2D(car->GetX(), car->GetZ(), kCarRadius, objectX, objectZ, objectRadius))
    {
        carHealth -= kHealthDecrease;
        if (carHealth < 0.0f) carHealth = 0.0f;

        float pushDistance = pushStrength * kPushDistance;
        float carX = car->GetX();
        float carZ = car->GetZ();

        float dx = carX - objectX;
        float dz = carZ - objectZ;
        float length = sqrt(dx * dx + dz * dz);
        if (length > 0)
        {
            dx /= length;
            dz /= length;
            car->SetX(carX + dx * pushDistance);
            car->SetZ(carZ + dz * pushDistance);
        }
        momentum = momentum * kMomentumDamping;
    }
}

void SwitchCameraView(ICamera* camera, IModel* car, CameraView view)
{
    if (view == CHASE_CAM)
    {
        camera->ResetOrientation();
        camera->SetLocalPosition(kCameraInitialX, kCameraInitialY, kCameraInitialZ);
        camera->AttachToParent(car);
    }
    else if (view == FIRST_PERSON)
    {
        camera->ResetOrientation();
        camera->SetLocalPosition(kCameraInitialX, kFirstPersonY, kFirstPersonZ);
        camera->AttachToParent(car);
    }
    else if (view == BIRD_VIEW)
    {
        camera->DetachFromParent();
        camera->ResetOrientation();
        float carX = car->GetX();
        float carZ = car->GetZ();
        camera->SetPosition(carX, 300.0f, carZ);
        camera->RotateX(90.0f);
    }
}

void GenerateTrackPoints(vector<Vector2D>& points)
{
    float baseRadius = kRoadRadius;
    float bump[12] = { 0, 10, 18, 10, 0, -10, -18, -10, 0, 10, 18, 10 };
    for (int i = 0; i < kNumTrackPoints; ++i)
    {
        float t = (2 * kPI * i) / (kNumTrackPoints - 1);
        float r = baseRadius + bump[i % 12];
        points.push_back({ r * sin(t), r * cos(t) });
    }
    points[points.size() - 1] = points[0];
}

float TrackLength(const vector<Vector2D>& points)
{
    float len = 0.0f;
    for (int i = 0; i < (int)points.size() - 1; ++i)
    {
        float dx = points[i + 1].x - points[i].x;
        float dz = points[i + 1].z - points[i].z;
        len += sqrt(dx * dx + dz * dz);
    }
    return len;
}

void GetTrackPosition(const vector<Vector2D>& points, float dist, float& x, float& z, float& angle)
{
    float total = 0.0f;
    for (int i = 0; i < (int)points.size() - 1; ++i)
    {
        float dx = points[i + 1].x - points[i].x;
        float dz = points[i + 1].z - points[i].z;
        float segLen = sqrt(dx * dx + dz * dz);
        if (dist <= total + segLen)
        {
            float t = (dist - total) / segLen;
            x = points[i].x + dx * t;
            z = points[i].z + dz * t;
            angle = atan2(dx, dz) * 180.0f / kPI;
            return;
        }
        total += segLen;
    }
    x = points[0].x;
    z = points[0].z;
    float dx = points[1].x - points[0].x;
    float dz = points[1].z - points[0].z;
    angle = atan2(dx, dz) * 180.0f / kPI;
}

void ResetGame(
    IModel* car, IModel* tank, vector<Checkpoint>& checkpoints, int& currentCheckpoint,
    float& carHealth, Vector2D& momentum, GameState& gameState, float& countdownTimer,
    const vector<Vector2D>& trackPoints, float trackLen,
    float tankX, float tankZ, float tankAngle, int& lap, bool& crossedStart)
{
    float carDist = 0.0f;
    float x, z, angle;
    GetTrackPosition(trackPoints, carDist, x, z, angle);
    car->SetPosition(x, 0.0f, z);
    car->ResetOrientation();
    car->RotateY(angle);

    tank->SetPosition(tankX, 0.0f, tankZ);
    tank->ResetOrientation();
    tank->RotateY(tankAngle);

    for (auto& cp : checkpoints) cp.passed = false;
    currentCheckpoint = 0;
    carHealth = kInitialHealth;
    momentum = { 0.0f, 0.0f };
    gameState = COUNTDOWN;
    countdownTimer = kCountdownDuration;
    lap = 1;
    crossedStart = false;
}

bool IsCarCrossingArch(IModel* car, const Vector2D& archPos, float archRadius = 8.0f)
{
    float dx = car->GetX() - archPos.x;
    float dz = car->GetZ() - archPos.z;
    return (dx * dx + dz * dz) < (archRadius * archRadius);
}

void main()
{
    I3DEngine* myEngine = New3DEngine(kTLX);
    myEngine->StartWindowed();
    myEngine->AddMediaFolder(".//Media");

    // Meshes
    IMesh* carMesh = myEngine->LoadMesh("race2.x");
    IMesh* barrierMesh = myEngine->LoadMesh("IsleStraight.x");
    IMesh* tankMesh = myEngine->LoadMesh("TankSmall1.x");
    IMesh* checkpointMesh = myEngine->LoadMesh("Checkpoint.x");
    IMesh* skyboxMesh = myEngine->LoadMesh("Skybox 07.x");
    IMesh* floorMesh = myEngine->LoadMesh("ground.x");

    // Track points
    vector<Vector2D> trackPoints;
    GenerateTrackPoints(trackPoints);
    float trackLen = TrackLength(trackPoints);

    // Models
    IModel* car = carMesh->CreateModel(0, 0, 0);
    ApplyScale(car, kCarScale);

    // Place the tank at a fixed position on the track (e.g., at 1/3 of the track)
    float tankT = 1.0f / 3.0f;
    int tankSeg = (int)(tankT * (trackPoints.size() - 1));
    float tankX = (trackPoints[tankSeg].x + trackPoints[tankSeg + 1].x) / 2.0f;
    float tankZ = (trackPoints[tankSeg].z + trackPoints[tankSeg + 1].z) / 2.0f;
    float tankAngle = atan2(trackPoints[tankSeg + 1].x - trackPoints[tankSeg].x, trackPoints[tankSeg + 1].z - trackPoints[tankSeg].z) * 180.0f / kPI;
    IModel* tank = tankMesh->CreateModel(tankX, 0.0f, tankZ);
    ApplyScale(tank, kTankScale);
    tank->ResetOrientation();
    tank->RotateY(tankAngle);

    IModel* skybox = skyboxMesh->CreateModel(0, -840.0f, 0);
    IModel* floor = floorMesh->CreateModel(0.0f, 0.0f, 0.0f);

    // --- BARRIERS AROUND THE ROAD (NO SPACE, LESS AT START ARCH) ---
    vector<IModel*> barriers;

    // Calculate start and end arch positions for gap
    float x0 = trackPoints[0].x;
    float z0 = trackPoints[0].z;
    float x1 = trackPoints[1].x;
    float z1 = trackPoints[1].z;
    int lastIdx = (int)trackPoints.size() - 2;
    float xEnd = trackPoints[lastIdx].x;
    float zEnd = trackPoints[lastIdx].z;
    float xEndNext = trackPoints[lastIdx + 1].x;
    float zEndNext = trackPoints[lastIdx + 1].z;

    // How close a barrier can be to the arch before being skipped (gap size)
    const float kArchGapRadius = 13.0f; // wider gap for arch
    const float kStartArchFewerRadius = 18.0f;

    for (int i = 0; i < (int)trackPoints.size() - 1; ++i)
    {
        float x0 = trackPoints[i].x;
        float z0 = trackPoints[i].z;
        float x1 = trackPoints[i + 1].x;
        float z1 = trackPoints[i + 1].z;

        float dx = x1 - x0;
        float dz = z1 - z0;
        float segLen = sqrt(dx * dx + dz * dz);
        if (segLen < 0.01f) continue;
        float nx = dz / segLen, nz = -dx / segLen;
        float segmentAngle = atan2(dx, dz) * 180.0f / kPI;

        // Determine if this segment is near the start arch
        float midx = (x0 + x1) / 2.0f;
        float midz = (z0 + z1) / 2.0f;
        float distToStartArch = sqrt((midx - trackPoints[0].x) * (midx - trackPoints[0].x) + (midz - trackPoints[0].z) * (midz - trackPoints[0].z));
        int barriersThisSegment = (distToStartArch < kStartArchFewerRadius) ? kBarriersPerSegmentStartArch : kBarriersPerSegment;

        for (int j = 0; j < barriersThisSegment; ++j)
        {
            float t = (float)j / (barriersThisSegment - 1);
            float px = x0 + dx * t;
            float pz = z0 + dz * t;

            // Don't place barriers if within the starting arch area (wider gap)
            float distToStart = sqrt((px - trackPoints[0].x) * (px - trackPoints[0].x) + (pz - trackPoints[0].z) * (pz - trackPoints[0].z));
            float distToEnd = sqrt((px - xEnd) * (px - xEnd) + (pz - zEnd) * (pz - zEnd));
            if (distToStart < kArchGapRadius || distToEnd < kArchGapRadius)
                continue;

            // Inner barrier
            float xIn = px - nx * ((kRoadWidth / 2.0f) + kBarrierGap);
            float zIn = pz - nz * ((kRoadWidth / 2.0f) + kBarrierGap);
            IModel* barrierIn = barrierMesh->CreateModel(xIn, 0.0f, zIn);
            barrierIn->ResetOrientation();
            barrierIn->RotateY(segmentAngle);
            ApplyScale(barrierIn, kBarrierScale);
            barriers.push_back(barrierIn);

            // Outer barrier
            float xOut = px + nx * ((kRoadWidth / 2.0f) + kBarrierGap);
            float zOut = pz + nz * ((kRoadWidth / 2.0f) + kBarrierGap);
            IModel* barrierOut = barrierMesh->CreateModel(xOut, 0.0f, zOut);
            barrierOut->ResetOrientation();
            barrierOut->RotateY(segmentAngle);
            ApplyScale(barrierOut, kBarrierScale);
            barriers.push_back(barrierOut);
        }
    }

    // --- START AND END ARCHES ACROSS THE ROAD, ALIGNED WITH THE TRACK DIRECTION ---
    vector<IModel*> arches;
    // Start arch: use the first segment to get direction and place at midpoint between 0 and 1
    float startArchX = (trackPoints[0].x + trackPoints[1].x) / 2.0f;
    float startArchZ = (trackPoints[0].z + trackPoints[1].z) / 2.0f;
    float startArchAngle = atan2(trackPoints[1].x - trackPoints[0].x, trackPoints[1].z - trackPoints[0].z) * 180.0f / kPI;
    IModel* startArch = barrierMesh->CreateModel(startArchX, 0.0f, startArchZ);
    startArch->ResetOrientation();
    startArch->RotateY(startArchAngle);
    ApplyScale(startArch, kBarrierScale);
    arches.push_back(startArch);

    // End arch: use the last segment to get direction and place at midpoint between last two points
    float endArchX = (trackPoints[lastIdx].x + trackPoints[lastIdx + 1].x) / 2.0f;
    float endArchZ = (trackPoints[lastIdx].z + trackPoints[lastIdx + 1].z) / 2.0f;
    float endArchAngle = atan2(trackPoints[lastIdx + 1].x - trackPoints[lastIdx].x, trackPoints[lastIdx + 1].z - trackPoints[lastIdx].z) * 180.0f / kPI;
    IModel* endArch = barrierMesh->CreateModel(endArchX, 0.0f, endArchZ);
    endArch->ResetOrientation();
    endArch->RotateY(endArchAngle);
    ApplyScale(endArch, kBarrierScale);
    arches.push_back(endArch);

    // --- TANKS IN THE MIDDLE OF THE ROAD, SPACED OUT ---
    vector<IModel*> tanks;
    int numTanks = 8;
    for (int i = 0; i < numTanks; ++i)
    {
        // Place tanks near the track center (midpoint between left/right edge)
        float t = (float)(i + 1) / (numTanks + 1);
        int seg = (int)(t * (trackPoints.size() - 1));
        float x0 = trackPoints[seg].x;
        float z0 = trackPoints[seg].z;
        float x1 = trackPoints[seg + 1].x;
        float z1 = trackPoints[seg + 1].z;
        // Position at center of segment
        float tankX = (x0 + x1) / 2.0f;
        float tankZ = (z0 + z1) / 2.0f;
        IModel* tankStatic = tankMesh->CreateModel(tankX, 0.0f, tankZ);
        ApplyScale(tankStatic, kTankScale);
        tanks.push_back(tankStatic);
    }

    // Checkpoints (4, spaced along the track)
    vector<Checkpoint> checkpoints;
    for (int i = 0; i < kNumCheckpoints; ++i)
    {
        float t = (float)i / kNumCheckpoints;
        int seg = (int)(t * (trackPoints.size() - 1));
        float x = trackPoints[seg].x;
        float z = trackPoints[seg].z;
        IModel* cpModel = checkpointMesh->CreateModel(x, 0.0f, z);
        ApplyScale(cpModel, 1.0f);
        BoundingBox box = { x - 6.0f, z - 6.0f, x + 6.0f, z + 6.0f };
        checkpoints.push_back({ cpModel, false, box });
    }

    // Game variables
    Vector2D momentum = { 0.0f, 0.0f };
    float carHealth = kInitialHealth;
    int currentCheckpoint = 0;
    GameState gameState = START_MENU;
    float countdownTimer = 0.0f;
    string gameStateMessage = "Welcome to Racing Game! Press SPACE to Begin";
    CameraView currentCameraView = CHASE_CAM;
    int lap = 1;
    bool crossedStart = false;

    // UI
    IFont* font = myEngine->LoadFont("Comic Sans MS", 36);
    ISprite* gameStateBackdrop = myEngine->CreateSprite("ui_backdrop.jpg", 0.0f, myEngine->GetHeight() - kGameStateTextOffsetY - 10, 0.0f);
    ISprite* statsBackdrop = myEngine->CreateSprite("ui_backdrop.jpg", 0.0f, myEngine->GetHeight() - kStatsTextOffsetY - 10, 0.0f);

    ICamera* camera = myEngine->CreateCamera(kManual, kCameraInitialX, kCameraInitialY, kCameraInitialZ);
    camera->AttachToParent(car);

    while (myEngine->IsRunning())
    {
        myEngine->DrawScene();
        float frameTime = myEngine->Timer();

        if (myEngine->KeyHit(Key_Escape))
            myEngine->Stop();

        switch (gameState)
        {
        case START_MENU:
            if (myEngine->KeyHit(Key_Space))
            {
                ResetGame(car, tank, checkpoints, currentCheckpoint, carHealth, momentum, gameState, countdownTimer, trackPoints, trackLen, tankX, tankZ, tankAngle, lap, crossedStart);
                gameStateMessage = "3 - GO!";
            }
            break;

        case COUNTDOWN:
            countdownTimer -= frameTime;
            if (countdownTimer <= 0)
            {
                gameState = RACING;
                gameStateMessage = "GO! Race to the first checkpoint!";
            }
            else if (countdownTimer <= 1)
                gameStateMessage = "1 - Go...";
            else if (countdownTimer <= 2)
                gameStateMessage = "2 - Set...";
            else
                gameStateMessage = "3 - Ready start the engines!";
            break;

        case RACING:
        {
            // --- PLAYER CAR ---
            if (myEngine->KeyHeld(Key_W))
            {
                Vector2D thrust = CalculateThrustVector(car, kThrustForce * frameTime);
                momentum = momentum + thrust;
            }
            else if (myEngine->KeyHeld(Key_S))
            {
                Vector2D thrust = CalculateThrustVector(car, kReverseThrustForce * frameTime);
                thrust = { -thrust.x, -thrust.z };
                momentum = momentum + thrust;
            }

            Vector2D drag = CalculateDragVector(momentum, frameTime);
            momentum = momentum + drag;

            float speed = sqrt(momentum.x * momentum.x + momentum.z * momentum.z);
            float direction = 1.0f;

            if (momentum.x != 0.0f || momentum.z != 0.0f)
            {
                Vector2D forward = CalculateThrustVector(car, 1.0f);
                float dotProduct = momentum.x * forward.x + momentum.z * forward.z;
                direction = (dotProduct >= 0.0f) ? 1.0f : -1.0f;
            }

            if (direction > 0.0f && speed > kMaxSpeed)
                momentum = momentum * (kMaxSpeed / speed);
            else if (direction < 0.0f && speed > kMaxReverseSpeed)
            {
                Vector2D reverseDir = momentum * (1.0f / speed);
                momentum = reverseDir * kMaxReverseSpeed;
            }

            if (myEngine->KeyHeld(Key_A))
                car->RotateY(-kTurnSpeed * frameTime);
            if (myEngine->KeyHeld(Key_D))
                car->RotateY(kTurnSpeed * frameTime);

            if (speed > kMinSpeedThreshold || speed < -kMinSpeedThreshold)
            {
                car->MoveLocalX(momentum.x * frameTime);
                car->MoveLocalZ(momentum.z * frameTime);
            }

            // --- COLLISIONS ---
            for (auto barrier : barriers)
                HandleCarCollision(car, barrier->GetX(), barrier->GetZ(), kBarrierRadius, carHealth, momentum, 1.5f);
            for (auto arch : arches)
                HandleCarCollision(car, arch->GetX(), arch->GetZ(), kBarrierRadius, carHealth, momentum, 1.5f);
            for (auto tankStatic : tanks)
                HandleCarCollision(car, tankStatic->GetX(), tankStatic->GetZ(), kTankRadius, carHealth, momentum, 2.0f);
            HandleCarCollision(car, tank->GetX(), tank->GetZ(), kTankRadius, carHealth, momentum, 2.0f);

            if (currentCheckpoint < checkpoints.size() && CheckPointCollision(car, checkpoints[currentCheckpoint]))
            {
                checkpoints[currentCheckpoint].passed = true;
                currentCheckpoint++;
                if (currentCheckpoint < checkpoints.size())
                    gameStateMessage = "Checkpoint " + to_string(currentCheckpoint) + " Cleared!";
                else
                    gameStateMessage = "Lap " + to_string(lap) + " complete! Cross the start arch!";
            }

            static bool wasInStart = false;
            Vector2D startArchPos = { startArchX, startArchZ };
            if (IsCarCrossingArch(car, startArchPos))
            {
                if (!wasInStart && currentCheckpoint == kNumCheckpoints)
                {
                    lap++;
                    currentCheckpoint = 0;
                    for (auto& cp : checkpoints) cp.passed = false;
                    if (lap > kTotalLaps)
                    {
                        gameState = FINISHED;
                        gameStateMessage = "Congratulations! You finished " + to_string(kTotalLaps) + " laps! Press R to replay.";
                    }
                    else
                        gameStateMessage = "Lap " + to_string(lap) + " started!";
                }
                wasInStart = true;
            }
            else
                wasInStart = false;

            if (carHealth <= 0.0f)
            {
                momentum = { 0.0f, 0.0f };
                gameStateMessage = "Game Over - Car Destroyed! Press R to restart";
                gameState = FINISHED;
            }
            break;
        }

        case FINISHED:
            if (myEngine->KeyHit(Key_R))
            {
                ResetGame(car, tank, checkpoints, currentCheckpoint, carHealth, momentum, gameState, countdownTimer, trackPoints, trackLen, tankX, tankZ, tankAngle, lap, crossedStart);
                gameStateMessage = "3 - GO!";
            }
            break;
        }

        if (gameState == RACING || gameState == FINISHED)
        {
            if (myEngine->KeyHit(Key_1))
            {
                currentCameraView = CHASE_CAM;
                SwitchCameraView(camera, car, currentCameraView);
            }
            else if (myEngine->KeyHit(Key_2))
            {
                currentCameraView = FIRST_PERSON;
                SwitchCameraView(camera, car, currentCameraView);
            }
            else if (myEngine->KeyHit(Key_3))
            {
                currentCameraView = BIRD_VIEW;
                SwitchCameraView(camera, car, currentCameraView);
            }

            if (currentCameraView == CHASE_CAM)
            {
                if (myEngine->KeyHeld(Key_Up)) camera->MoveLocalZ(kCameraSpeed * frameTime);
                if (myEngine->KeyHeld(Key_Down)) camera->MoveLocalZ(-kCameraSpeed * frameTime);
                if (myEngine->KeyHeld(Key_Right)) camera->MoveLocalX(kCameraSpeed * frameTime);
                if (myEngine->KeyHeld(Key_Left)) camera->MoveLocalX(-kCameraSpeed * frameTime);
                int mouseX = myEngine->GetMouseMovementX();
                if (mouseX != 0)
                {
                    camera->RotateY(mouseX * frameTime * kMouseSensitivity);
                    camera->RotateY(mouseX * frameTime * kMouseSensitivity);
                }
            }
            else if (currentCameraView == FIRST_PERSON)
            {
                int mouseX = myEngine->GetMouseMovementX();
                int mouseY = myEngine->GetMouseMovementY();
                if (mouseX != 0)
                {
                    camera->RotateY(mouseX * frameTime * kMouseSensitivity);
                    camera->RotateY(mouseX * frameTime * kMouseSensitivity);
                }
                if (mouseY != 0)
                {
                    float currentRotationX = camera->GetLocalX();
                    float newRotationX = currentRotationX + (mouseY * frameTime * kMouseSensitivity);
                    if (newRotationX > -kMaxPitchAngle && newRotationX < kMaxPitchAngle)
                        camera->RotateLocalX(mouseY * frameTime * kMouseSensitivity);
                }
            }
            else if (currentCameraView == BIRD_VIEW)
            {
                float carX = car->GetX();
                float carZ = car->GetZ();
                camera->SetPosition(carX, 300.0f, carZ);
                camera->ResetOrientation();
                camera->RotateX(90.0f);
            }
        }

        font->Draw(gameStateMessage, kGameStateTextX, myEngine->GetHeight() - kGameStateTextOffsetY, kBlue);
        float speedMps = sqrt(momentum.x * momentum.x + momentum.z * momentum.z);
        float speedMph = speedMps / kMphToMps;
        string speedText = "Speed: " + to_string(int(speedMph)) + " mph";
        font->Draw(speedText, kGameStateTextX, myEngine->GetHeight() - kStatsTextOffsetY, kRed);
        string healthText = "Health: " + to_string(int(carHealth));
        if (carHealth <= kLowHealthThreshold)
            font->Draw(healthText, kHealthTextX, myEngine->GetHeight() - kStatsTextOffsetY, kRed);
        else
            font->Draw(healthText, kHealthTextX, myEngine->GetHeight() - kStatsTextOffsetY, kGreen);

        string lapText = "Lap: " + to_string(min(lap, kTotalLaps)) + "/" + to_string(kTotalLaps);
        font->Draw(lapText, kHealthTextX + 200, myEngine->GetHeight() - kStatsTextOffsetY, kYellow);
    }

    delete gameStateBackdrop;
    delete statsBackdrop;
    myEngine->Delete();
}
