#include "Robot/Robot.h"
#include <Utils.h>
#include <Arduino.h>
#include <Config.h>
#if defined(ARDUINO_ARCH_AVR)
#include <avr/interrupt.h>
#endif

Robot* Robot::instance = nullptr;

#if defined(ARDUINO_ARCH_AVR)
ISR(TIMER1_COMPA_vect)
{
    Robot::stepTimerISR();
}
#endif

Robot::Robot(Logger& logger, missionManager *missionManager) : m_logger(logger), m_missionManager(missionManager)
{
    pinMode(config::M_EN_PIN, OUTPUT);
    digitalWrite(config::M_EN_PIN, HIGH); // free motors
    if (config::MOTOR_ODOM_ONLY) {
        motor_left = new Motor(config::M1_DIR_PIN,
                                config::M1_STEP_PIN,
                                -1,
                                false,
                                config::M1_INVERT_DIR,
                                config::M1_WHEEL_DIAMETER_MM * M_PI / 1000.0,
                                Robot::leftMotorStepNotify);

        motor_right = new Motor(config::M2_DIR_PIN,
                                config::M2_STEP_PIN,
                                -1,
                                true,
                                config::M2_INVERT_DIR,
                                config::M2_WHEEL_DIAMETER_MM * M_PI / 1000.0,
                                Robot::rightMotorStepNotify);
    }
    else {
        motor_left = new Motor(config::M1_DIR_PIN,
                                config::M1_STEP_PIN,
                                config::OD1_CS_PIN,
                                false,
                                config::M1_INVERT_DIR,
                                config::M1_WHEEL_DIAMETER_MM * M_PI / 1000.0,
                                Robot::leftMotorStepNotify);

        motor_right = new Motor(config::M2_DIR_PIN,
                                config::M2_STEP_PIN,
                                config::OD2_CS_PIN,
                                true,
                                config::M2_INVERT_DIR,
                                config::M2_WHEEL_DIAMETER_MM * M_PI / 1000.0,
                                Robot::rightMotorStepNotify);
    }
    Robot::instance = this;

#if defined(ARDUINO_ARCH_AVR)
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    // Timer1 CTC with prescaler /8.
    unsigned long ocr = (F_CPU / (8UL * (unsigned long)config::MOTOR_TIMER_FREQUENCY_HZ)) - 1UL;
    if (ocr > 65535UL) {
        ocr = 65535UL;
    }
    OCR1A = (uint16_t)ocr;
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11);
    TIMSK1 |= (1 << OCIE1A);
    interrupts();
#endif
}

void Robot::run() {
#if defined(ARDUINO_ARCH_AVR)
    // Motor stepping is driven by Timer1 ISR on AVR.
#else
    motor_left->run();
    motor_right->run();
#endif
    Control();
}

void Robot::stepTimerISR()
{
    if (instance == nullptr) {
        return;
    }
    instance->motor_left->runFromTimerISR();
    instance->motor_right->runFromTimerISR();
}

void Robot::resetOdometry() {
    x = 0.0;
    y = 0.0;
    theta = 0.0;

    m_motorX = 0.0;
    m_motorY = 0.0;
    m_motorTheta = 0.0;

    m_lastLinearSpeed = 0.0;
    m_lastAngularSpeed = 0.0;
    m_linearSpeedMotor = 0.0;
    m_angularSpeedMotor = 0.0;
    m_currentLinearAccel = 0.0;
    m_leftStepCount = 0;
    m_rightStepCount = 0;
}

void Robot::Control()
{
    unsigned long long currentTime = micros();

    if (!config::MOTOR_ODOM_ONLY && currentTime - m_lastOdomTime > 1e6/config::ODOM_MEASURE_FREQUENCY_HZ) {
        updateOdometry();
        m_lastOdomTime = currentTime;
    }

    if (currentTime - m_lastControlTime >= 1e6/config::CONTROL_LOOP_FREQUENCY_HZ) {
        // Plafonner m_dt pour éviter une explosion au premier tick ou après une longue pause
        
        m_dt = (unsigned int)(currentTime - m_lastControlTime);
        // unsigned long long elapsed = currentTime - m_lastControlTime;
        // unsigned int const maxDt = (unsigned int)(2.0f * 1e6f / config::CONTROL_LOOP_FREQUENCY_HZ);
        // m_dt = (elapsed > maxDt) ? maxDt : (unsigned int)elapsed;
        m_lastControlTime = currentTime;
        if (config::MOTOR_ODOM_ONLY) {
            updateMotorOdometry();
        }
        if (m_isEmergencyStop) {
            setEmergencyStopMotorSpeed();
            return;
        }
        bool hasActiveMission = m_missionManager->hasActiveMissions();
        if (!hasActiveMission && m_missionManager->hasMissions())
        {
            m_missionManager->startNextMission();
            hasActiveMission = m_missionManager->hasActiveMissions();
            if (hasActiveMission && m_missionManager->getCurrentMission()->getType() == Mission::Type::GO) {
                Mission* CurrentMission = m_missionManager->getCurrentMission();
                float const dx = m_missionManager->getCurrentMission()->getTargetX() - getX();
                float const dy = m_missionManager->getCurrentMission()->getTargetY() - getY();
                float const angle_cible = atan2(dy, dx);
                CurrentMission->setThetaGoTo(angle_cible);
            }
        }
        if (hasActiveMission)
        {
            m_lastAssignedMission = currentTime;
            digitalWrite(config::M_EN_PIN, LOW);
            checkMissionArrived();
            Mission* CurrentMission = m_missionManager->getCurrentMission();
            if (CurrentMission == nullptr) {
                stop();
                return;
            }
            else if (CurrentMission->isActive() == false) {
                Mission* nextMission = m_missionManager->startNextMission();
                if (nextMission == nullptr) {
                    stop();
                    return;
                }
                float const dx = nextMission->getTargetX() - getX();
                float const dy = nextMission->getTargetY() - getY();
                float const angle_cible = atan2(dy, dx);
                nextMission->setThetaGoTo(angle_cible);
            }

            if (CurrentMission->getType() == Mission::Type::GO)
                samsonUpdateMotors(CurrentMission);
            else if (CurrentMission->getType() == Mission::Type::TURN)
                rotationUpdateMotors();
            else if (CurrentMission->getType() == Mission::Type::STOP || CurrentMission->getType() == Mission::Type::WAIT)
                stop();

            //m_logger.debug("Robot Position - X: " + String(getX(), 4) + " m, Y: " + String(getY(), 4) + " m, Theta: " + String(getTheta(), 4) + " rad");
            //m_logger.debug("Robot Speed - Linear: " + String(getLinearSpeed(), 4) + " m/s, Angular: " + String(getAngularSpeed(), 4) + " rad/s");
        }
        else {
            // Libérer les moteurs après 10s d'inactivité
            if (currentTime - m_lastAssignedMission > 10ULL * 1000000ULL) {
                digitalWrite(config::M_EN_PIN, HIGH);
            }
        }
    }
}

void Robot::updateOdometry() {
    unsigned int dt1, dt2, t;
    double right_speed = this->motor_right->getFeedbackSpeed(&dt1, &t);
    double left_speed = this->motor_left->getFeedbackSpeed(&dt2, &t);

    m_dt = (dt1+dt2)/2;
    m_t = t;

    m_linearSpeed = (left_speed + right_speed)/2;
    m_angularSpeed = (right_speed-left_speed)/(config::OD_WHEEL_BASE_MM/1000.0);

    // Serial.print("Left speed: " + String(left_speed) + " m/s, Right speed: " + String(right_speed) + " m/s ");
    // Serial.println("=> Linear speed: " + String(m_linearSpeed) + " m/s, Angular speed: " + String(m_angularSpeed) + " rad/s");

    // integration selon la methode des trapezes
    float angular_speed = (0.5*m_angularSpeed+0.5*m_lastAngularSpeed);
    float linear_speed = (0.5*m_linearSpeed+0.5*m_lastLinearSpeed);
    float dTheta = angular_speed * m_dt / (float)1e6;
    if (abs(dTheta) > 1e-6f) {
        // Méthode de l'arc de cercle
        float R = linear_speed / angular_speed;
        x += R * (sin(theta + dTheta) - sin(theta));
        y += -R * (cos(theta + dTheta) - cos(theta));
    } else {
        // Méthode de la droite
        x += linear_speed * cos(theta) * m_dt / (float)1e6;
        y += linear_speed * sin(theta) * m_dt / (float)1e6;
    }
    theta = utils::normalizeAngle(theta + dTheta);
    m_lastLinearSpeed = m_linearSpeed;
    m_lastAngularSpeed = m_angularSpeed;
}

void Robot::emergencyStop(bool enable)
{
    if (enable)
        m_missionManager->cancelAllMissions();
    m_isEmergencyStop = enable;
}

void Robot::setSpeeds(float linearSpeed, float angularSpeed)
{
    float v_right = linearSpeed + (angularSpeed*config::M_WHEEL_BASE_MM/1000.0)/2.0f;
    float v_left = linearSpeed - (angularSpeed*config::M_WHEEL_BASE_MM/1000.0)/2.0f;

    motor_right->setLinearSpeed(v_right);
    motor_left->setLinearSpeed(v_left);
}

void Robot::setEmergencyStopMotorSpeed()
{
    if (m_angularSpeedMotor > 0) {
        m_angularSpeedMotor = utils::getMax(m_angularSpeedMotor - config::ANGULAR_EMERGENCY_MAX_DECELEATION * (m_dt / (float)1e6), 0.0f);
    } else if (m_angularSpeedMotor < 0) {
        m_angularSpeedMotor = utils::getMin(m_angularSpeedMotor + config::ANGULAR_EMERGENCY_MAX_DECELEATION * (m_dt / (float)1e6), 0.0f);
    }
    
    if (m_linearSpeedMotor > 0) {
        m_linearSpeedMotor = utils::getMax(m_linearSpeedMotor - config::LINEAR_EMERGENCY_MAX_DECELEATION * (m_dt / (float)1e6), 0.0f);
    } else if (m_linearSpeedMotor < 0) {
        m_linearSpeedMotor = utils::getMin(m_linearSpeedMotor + config::LINEAR_EMERGENCY_MAX_DECELEATION * (m_dt / (float)1e6), 0.0f);
    }
    m_lastLinearSpeed = m_linearSpeed;
    m_lastAngularSpeed = m_angularSpeed;
    setSpeeds(m_linearSpeedMotor, m_angularSpeedMotor);
}

void Robot::samsonUpdateMotors(Mission* mission)
{
    float const dt_s = m_dt / (float)1e6;

    float const targetX     = mission->getTargetX();
    float const targetY     = mission->getTargetY();
    float const angle_cible = mission->getTargetTheta();
    bool  const forward     = mission->isForward();

    // ================================================================
    // 1. ERREURS DE TRAJECTOIRE
    // ================================================================
    float const dx = getX() - targetX;
    float const dy = getY() - targetY;
    float const distance = sqrt(dx * dx + dy * dy);

    // Erreur latérale (perpendiculaire à la droite de référence)
    float const lateral_error = dx * sin(angle_cible) - dy * cos(angle_cible);

    // Distance longitudinale signée (le long de la droite de référence)
    // Négative → le robot est derrière la cible ; positive → il l'a dépassée
    float const longitudinal_distance = dx * cos(angle_cible) + dy * sin(angle_cible);

    // Distance restante (toujours >= 0)
    float const remaining_distance = utils::getMax(-longitudinal_distance, 0.0f);

    // Erreur angulaire
    // En marche arrière l'arrière du robot doit pointer vers angle_cible
    float theta_error;
    if (forward)
        theta_error = angle_cible - getTheta();
    else
        theta_error = angle_cible - getTheta() - M_PI;
    theta_error = utils::normalizeAngle(theta_error);

    // ================================================================
    // 2. CONTRÔLE ANGULAIRE — suivi de trajectoire (Samson)
    // ================================================================
    float const kp_angular = 5.0f;   // gain correction cap
    float const kp_lateral = 2.0f;   // gain correction latérale
    float const k_damping  = 0.5f;   // amortissement latéral

    // En marche arrière l'effet de ω sur la déviation latérale est inversé
    float const lateral_sign = forward ? -1.0f : 1.0f;

    float omega_desired = kp_angular * theta_error
                        + lateral_sign * kp_lateral * lateral_error
                        + lateral_sign * k_damping * lateral_error
                          * abs(longitudinal_distance) / (distance + 0.01f);

    // Saturer ω_desired
    omega_desired = utils::getMin(
        utils::getMax(omega_desired, -config::MAX_ANGULAR_VELOCITY_RAD_S),
        config::MAX_ANGULAR_VELOCITY_RAD_S);

    // Lisser l'accélération angulaire
    float omega = m_angularSpeedMotor;
    float const omega_diff      = omega_desired - omega;
    float const max_omega_change = config::ANGULAR_ACCELERATION_RAD_S2 * dt_s;
    if (abs(omega_diff) > max_omega_change)
        omega += utils::sign(omega_diff) * max_omega_change;
    else
        omega = omega_desired;

    // ================================================================
    // 3. CONTRÔLE LINÉAIRE — profil S-curve
    //    (jerk limité, accélération / décélération asymétriques)
    // ================================================================
    float const a_acc   = config::LINEAR_ACCELERATION_M_S2;   // accélération max
    float const a_dec   = config::LINEAR_DECCELERATION_M_S2;  // décélération max
    float const j_max   = config::LINEAR_JERK_M_S3;           // jerk max (accel ET decel)
    float const v_limit = config::MAX_LINEAR_VELOCITY_M_S;    // vitesse linéaire max

    // --- 3a. Vitesse max admissible pour freiner à temps ----------------
    // Distance de freinage (profil trapézoïdal en accélération, jerk limité) :
    //   d = v²/(2·a_dec) + v·a_dec/(2·j)
    // Résolution de l'équation quadratique pour v :
    //   (1/(2·a_dec))·v² + (a_dec/(2·j))·v − d = 0
    float const braking_margin = utils::getMax(
        remaining_distance - config::GO_MISSION_TOLERANCE_M * 0.5f, 0.0f);
    float const A_b  = 1.0f / (2.0f * a_dec);
    float const B_b  = a_dec / (2.0f * j_max);
    float const disc = B_b * B_b + 4.0f * A_b * braking_margin;
    float v_max_braking;
    if (disc > 0.0f)
        v_max_braking = (-B_b + sqrt(disc)) / (2.0f * A_b);
    else
        v_max_braking = 0.0f;
    v_max_braking = utils::getMin(v_max_braking, v_limit);

    // --- 3b. Réduction douce pour erreur de cap / latérale --------------
    // Facteurs très progressifs pour ne pas créer d'à-coups au début de la décélération
    float const heading_slowdown = 1.0f / (1.0f + 0.3f * abs(theta_error));
    float const lateral_slowdown = 1.0f / (1.0f + 0.4f * abs(lateral_error));
    float const v_max_soft = v_max_braking
                           * utils::getMax(0.6f, heading_slowdown * lateral_slowdown);

    // Vitesse cible signée (cible douce pour le jerk limiter)
    float const target_speed = forward ? v_max_soft : -v_max_soft;

    // --- 3c. Limites d'accélération asymétriques ------------------------
    // En marche avant  : accel > 0 → accélération, accel < 0 → décélération
    // En marche arrière : accel < 0 → accélération, accel > 0 → décélération
    float a_pos_limit, a_neg_limit;
    if (forward) {
        a_pos_limit =  a_acc;   // accélération (v augmente)
        a_neg_limit = -a_dec;   // décélération (v diminue)
    } else {
        a_pos_limit =  a_dec;   // décélération (|v| diminue)
        a_neg_limit = -a_acc;   // accélération (|v| augmente)
    }

    // --- 3d. Consigne d'accélération (P sur l'erreur de vitesse) --------
    float const v_error  = target_speed - m_linearSpeedMotor;
    float const kp_accel = 10.0f;
    float accel_desired  = kp_accel * v_error;
    accel_desired = utils::getMin(utils::getMax(accel_desired, a_neg_limit), a_pos_limit);

    // --- 3e. Jerk limiter — variation de l'accélération bornée ----------
    float const max_jerk_change = j_max * dt_s;
    float const accel_diff = accel_desired - m_currentLinearAccel;
    if (abs(accel_diff) > max_jerk_change)
        m_currentLinearAccel += utils::sign(accel_diff) * max_jerk_change;
    else
        m_currentLinearAccel = accel_desired;

    // Saturer aux limites asymétriques
    m_currentLinearAccel = utils::getMin(
        utils::getMax(m_currentLinearAccel, a_neg_limit), a_pos_limit);

    // --- 3f. Intégration de la vitesse ----------------------------------
    float v = m_linearSpeedMotor + m_currentLinearAccel * dt_s;

    // --- 3g. Clamp dur — distance de freinage + direction ---------------
    if (forward) {
        v = utils::getMax(v, 0.0f);
        v = utils::getMin(v, v_max_braking);
    } else {
        v = utils::getMin(v, 0.0f);
        v = utils::getMax(v, -v_max_braking);
    }

    // --- 3h. Resynchroniser l'état d'accélération -----------------------
    // Évite que le jerk limiter reste en avance sur la vitesse réellement appliquée
    if (dt_s > 1e-9f) {
        float const implied_accel = (v - m_linearSpeedMotor) / dt_s;
        m_currentLinearAccel = utils::getMin(
            utils::getMax(implied_accel, a_neg_limit), a_pos_limit);
    }

    // ================================================================
    // 4. ARRÊT FINAL
    // ================================================================
    if (remaining_distance < config::GO_MISSION_TOLERANCE_M / 4.0f && abs(v) < 0.02f) {
        v     = 0.0f;
        omega = 0.0f;
        m_currentLinearAccel = 0.0f;
    }

    // Près de la cible : ramener ω vers 0 progressivement
    if (remaining_distance < 2.0f * config::GO_MISSION_TOLERANCE_M) {
        float const max_omega_final = config::ANGULAR_ACCELERATION_RAD_S2 * dt_s;
        if (abs(omega) > max_omega_final)
            omega -= utils::sign(omega) * max_omega_final;
        else
            omega = 0.0f;
    }

    m_linearSpeedMotor  = v;
    m_angularSpeedMotor = omega;
    setSpeeds(v, omega);
}

void Robot::rotationUpdateMotors()
{
    float target_theta = m_missionManager->getCurrentMission()->getTargetTheta();
    bool const forward = m_missionManager->getCurrentMission()->isForward();

    if (!forward) {
        target_theta = utils::normalizeAngle(target_theta + M_PI);
    }

    float angle_diff = utils::normalizeAngle(target_theta - getTheta());
    float omega_desired;
    if (abs(angle_diff) < 0.01) { // Si l'angle est déjà atteint, arrêter le robot
        motor_left->stop();
        motor_right->stop();
        m_angularSpeedMotor = 0.0f;
        return;
    }
    // Calcul de la distance d'arrêt
    float stop_distance = (m_angularSpeedMotor * m_angularSpeedMotor) / (2.0f * config::ANGULAR_ACCELERATION_RAD_S2);
    // Calcul de la vitesse angulaire souhaitée
    if (abs(angle_diff) < stop_distance) {
        omega_desired = utils::sign(angle_diff) * sqrt(2.0f * config::ANGULAR_ACCELERATION_RAD_S2 * abs(angle_diff));
    } else {
        omega_desired = utils::sign(angle_diff) * config::MAX_ANGULAR_VELOCITY_RAD_S;
    }
    // Lissage de l'accélération angulaire
    float omega = m_angularSpeedMotor + utils::sign(omega_desired - m_angularSpeedMotor) * config::ANGULAR_ACCELERATION_RAD_S2 * (m_dt / (float)1e6);
    omega = utils::sign(omega_desired) * utils::getMin(abs(omega), abs(omega_desired));
    m_angularSpeedMotor = omega;
    m_linearSpeedMotor = 0.0;
    setSpeeds(0.0f, omega);
}

void Robot::stop()
{
    motor_left->stop();
    motor_right->stop();
    m_linearSpeedMotor = 0.0;
    m_angularSpeedMotor = 0.0;
    m_currentLinearAccel = 0.0;
    m_lastAngularSpeed = 0.0;
    m_lastLinearSpeed = 0.0;
}

bool Robot::checkMissionArrived()
{
    Mission* currentMission = m_missionManager->getCurrentMission();
    if (currentMission->getType() == Mission::Type::GO) {
        float const angle_cible = currentMission->getTargetTheta();
        float const longitudinal_distance = (getX() - currentMission->getTargetX()) * cos(angle_cible)
                                           + (getY() - currentMission->getTargetY()) * sin(angle_cible);
        float const remaining_distance = utils::getMax(-longitudinal_distance, 0.0f);
        if (remaining_distance < config::GO_MISSION_TOLERANCE_M) {
            m_missionManager->endCurrentMission();
            m_angularSpeedMotor = 0.0;
            m_linearSpeedMotor = 0.0;
            m_currentLinearAccel = 0.0;
            return true;
        }
    } else if (currentMission->getType() == Mission::Type::TURN) {
        float const target_theta = currentMission->getTargetTheta();
        float angle_diff = utils::normalizeAngle(target_theta - getTheta());
        if (abs(angle_diff) < config::TURN_MISSION_TOLERANCE_RAD) {
            m_missionManager->endCurrentMission();
            m_angularSpeedMotor = 0.0;
            m_linearSpeedMotor = 0.0;
            m_currentLinearAccel = 0.0;
            return true;
        }
    }
    return false;
}

void Robot::parseOdometryData(String const &message)
{
    // Message format: "Ox.x;y.y;theta.t" or "Ox.x;y.y;theta.tF"
    String data = message.substring(1); // Remove 'O' prefix
    int firstSemicolon = data.indexOf(';');
    int secondSemicolon = data.indexOf(';', firstSemicolon + 1);

    m_motorX = data.substring(0, firstSemicolon).toFloat();
    m_motorY = data.substring(firstSemicolon + 1, secondSemicolon).toFloat();
    m_motorTheta = utils::normalizeAngle(data.substring(secondSemicolon + 1).toFloat());

    x = m_motorX;
    y = m_motorY;
    theta = m_motorTheta;
}

void Robot::leftMotorStepNotify(bool forward)
{
    if (instance == nullptr) return;
    if (forward) instance->m_leftStepCount++;
    else instance->m_leftStepCount--;
}

void Robot::rightMotorStepNotify(bool forward)
{
    if (instance == nullptr) return;
    if (forward) instance->m_rightStepCount++;
    else instance->m_rightStepCount--;
}

void Robot::updateMotorOdometry()
{
    // Lire et remettre à zéro les compteurs de pas (section critique)
    noInterrupts();
    long leftSteps = m_leftStepCount;
    long rightSteps = m_rightStepCount;
    m_leftStepCount = 0;
    m_rightStepCount = 0;
    interrupts();

    // Calculer la distance parcourue par chaque roue
    float const stepsPerRev = config::STEP_PER_REVOLUTION * config::MICROSTEPS;
    float distLeft = leftSteps * motor_left->getWheelPerimeter() / stepsPerRev;
    float distRight = rightSteps * motor_right->getWheelPerimeter() / stepsPerRev;

    // Cinématique différentielle
    float dCenter = (distLeft + distRight) / 2.0f;
    float dTheta = (distRight - distLeft) / (config::M_WHEEL_BASE_MM / 1000.0f);

    // Mise à jour de la position par arc de cercle ou ligne droite
    if (abs(dTheta) > 1e-6f) {
        float R = dCenter / dTheta;
        m_motorX += R * (sin(m_motorTheta + dTheta) - sin(m_motorTheta));
        m_motorY += -R * (cos(m_motorTheta + dTheta) - cos(m_motorTheta));
    } else {
        m_motorX += dCenter * cos(m_motorTheta);
        m_motorY += dCenter * sin(m_motorTheta);
    }
    m_motorTheta = utils::normalizeAngle(m_motorTheta + dTheta);
}