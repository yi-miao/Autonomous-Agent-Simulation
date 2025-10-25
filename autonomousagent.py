import pygame, random, math
import numpy as np

# Constants
WIDTH, HEIGHT = 800, 600
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
DOT_RADIUS = 8
STD_DEV = 30
PULL_FACTOR = 0.05
RED = (255, 0, 0)
BLUE = (0, 100, 255)
GREEN = (0, 200, 0)
WHITE = (255, 255, 255)
    
def clamp(value, min_val, max_val):
    return max(min(value, max_val), min_val)
    
# InferenceEngine: a PID controller
class InferenceEngine:
    def __init__(self, kp=0.2, ki=0.01, kd=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# Environment class: A red spot moving randomly and continuesly
class Environment(pygame.sprite.Sprite):
    def __init__(self, center, radius, std_dev, pull_factor):
        super().__init__()
        self.image = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
        pygame.draw.circle(self.image, RED, (radius, radius), radius)
        self.rect = self.image.get_rect(center=center)
        self.center_x, self.center_y = center
        self.std_dev = std_dev
        self.pull_factor = pull_factor
        self.radius = radius

    def update(self):
        dx = self.center_x - self.rect.centerx
        dy = self.center_y - self.rect.centery
        pull_x = dx * self.pull_factor
        pull_y = dy * self.pull_factor
        dx_random = random.gauss(0, self.std_dev)
        dy_random = random.gauss(0, self.std_dev)
        new_x = self.rect.centerx + dx_random + pull_x
        new_y = self.rect.centery + dy_random + pull_y
        self.rect.center = (new_x, new_y)

# Perceptor: filter out noise with a Kalman filter
class Perceptor:
    def __init__(self):
        self.x = np.array([[CENTER_X], [CENTER_Y], [0], [0]], dtype=float)
        self.F = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=float)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=float)
        self.P = np.eye(4) * 500
        self.Q = np.eye(4) * 0.1
        self.R = np.eye(2) * 100

    def update(self, measured_x, measured_y):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        z = np.array([[measured_x], [measured_y]])
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z - self.H @ self.x
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        return int(self.x[0, 0]), int(self.x[1, 0])

# Effector: with InferenceEngine or PID control
class Effector(pygame.sprite.Sprite):
    def __init__(self, start_pos, radius, sensor):
        super().__init__()
        self.image = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
        pygame.draw.circle(self.image, GREEN, (radius, radius), radius)
        self.rect = self.image.get_rect(center=start_pos)
        self.sensor = sensor
        self.radius = radius
        self.pid_x = InferenceEngine(kp=0.2, ki=0.01, kd=0.1)
        self.pid_y = InferenceEngine(kp=0.2, ki=0.01, kd=0.1)

    def update(self, dt):
        target_x, target_y = self.sensor.x[0, 0], self.sensor.x[1, 0]
        current_x, current_y = self.rect.center
        error_x = target_x - current_x
        error_y = target_y - current_y
        distance = math.hypot(error_x, error_y)
        print(f"Distance to target: {distance:.2f}")

        move_x = clamp(self.pid_x.update(error_x, dt), -50, 50)
        move_y = clamp(self.pid_y.update(error_y, dt), -50, 50)
        self.rect.centerx += int(move_x)
        self.rect.centery += int(move_y)

# AutonomousAgent wrapper
class AutonomousAgent:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Autonomous Agent")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 36)
        self.impact_marker = pygame.image.load("assets/impact_marker.png").convert_alpha()
        self.impact_marker = pygame.transform.scale(self.impact_marker, (200, 200))
        self.sensor = Perceptor()
        self.attacker = Environment(center=(CENTER_X, CENTER_Y),
                                 radius=DOT_RADIUS,
                                 std_dev=STD_DEV,
                                 pull_factor=PULL_FACTOR)
        self.interceptor = Effector(start_pos=(100, 100),
                                       radius=DOT_RADIUS,
                                       sensor=self.sensor)
        self.all_sprites = pygame.sprite.Group(self.attacker, self.interceptor)
        self.start_time = pygame.time.get_ticks()
        self.game_over = False
        self.elapsed_seconds = 0
        self.impact_marker_rect = None
        self.hit_history = []
        
        self.show_traces = True
        self.raw_history = []
        self.filtered_history = []
        self.error_history = []
        self.output_history = []
        self.interceptor_history = []

    def reset(self):
        if self.elapsed_seconds > 0:
            self.hit_history.append(self.elapsed_seconds)

        self.sensor = Perceptor()
        self.attacker = Environment(center=(CENTER_X, CENTER_Y),
                                 radius=DOT_RADIUS,
                                 std_dev=STD_DEV,
                                 pull_factor=PULL_FACTOR)
        self.interceptor = Effector(start_pos=(100, 100),
                                       radius=DOT_RADIUS,
                                       sensor=self.sensor)
        self.all_sprites = pygame.sprite.Group(self.attacker, self.interceptor)
        self.start_time = pygame.time.get_ticks()
        self.game_over = False
        self.elapsed_seconds = 0
        self.impact_marker_rect = None
            
    def run(self):
        while True:
            dt = max(self.clock.get_time() / 1000.0, 1e-3)  # seconds

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN and self.game_over:
                        self.reset()
                    elif event.key == pygame.K_SPACE:
                        self.show_traces = not self.show_traces
                        print(f"Trace display toggled: {'ON' if self.show_traces else 'OFF'}")

            if not self.game_over:
                self.attacker.update()
                self.interceptor.update(dt)
                self.interceptor_history.append(self.interceptor.rect.center)

                measured_x, measured_y = self.attacker.rect.center
                filtered_x, filtered_y = self.sensor.update(measured_x, measured_y)

                # 🧠 Log analysis data
                self.raw_history.append((measured_x, measured_y))
                self.filtered_history.append((filtered_x, filtered_y))

                error_x = filtered_x - self.interceptor.rect.centerx
                error_y = filtered_y - self.interceptor.rect.centery
                self.error_history.append((error_x, error_y))

                output_x = self.interceptor.pid_x.prev_error
                output_y = self.interceptor.pid_y.prev_error
                self.output_history.append((output_x, output_y))

                if pygame.sprite.collide_circle(self.attacker, self.interceptor):
                    self.game_over = True
                    hit_time = pygame.time.get_ticks()
                    self.elapsed_seconds = (hit_time - self.start_time) / 1000
                    self.impact_marker_rect = self.impact_marker.get_rect(center=self.attacker.rect.center)
                    print(f"Target hit! Time used: {self.elapsed_seconds:.2f} seconds")

            self.screen.fill(WHITE)
            self.all_sprites.draw(self.screen)

            # 🔵 Draw filtered target position
            pygame.draw.circle(self.screen, BLUE, (filtered_x, filtered_y), DOT_RADIUS // 2)

            if self.show_traces:
                if len(self.raw_history) > 1:
                    pygame.draw.lines(self.screen, RED, False, self.raw_history[-100:], 2)
                if len(self.filtered_history) > 1:
                    pygame.draw.lines(self.screen, BLUE, False, self.filtered_history[-100:], 2)
                if len(self.interceptor_history) > 1:
                    pygame.draw.lines(self.screen, GREEN, False, self.interceptor_history[-100:], 2)

            if self.game_over:
                self.screen.blit(self.impact_marker, self.impact_marker_rect)
                text = self.font.render(f"Target hit in {self.elapsed_seconds:.2f} seconds", True, (0, 0, 0))
                self.screen.blit(text, (WIDTH // 2 - text.get_width() // 2, HEIGHT // 2 + 50))

            # 🕒 Show hit history
            if self.hit_history:
                for i, t in enumerate(reversed(self.hit_history[-5:])):
                    text = self.font.render(f"{t:.2f}s", True, (50, 50, 50))
                    self.screen.blit(text, (WIDTH - 120, 20 + i * 30))

            pygame.display.flip()
            self.clock.tick(30)

# Run the simulation
if __name__ == "__main__":
    AutonomousAgent().run()