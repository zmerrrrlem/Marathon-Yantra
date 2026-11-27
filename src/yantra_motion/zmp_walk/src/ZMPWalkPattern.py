'''
    Implementation of ZMP-based walking pattern generation.
    However, the offset tracking error of ZMP in a long distance walking pattern is observed in this method.
    For more details, refer to "Introduction to Humanoid Robotics" by Shuuji Kajita.

    Background: 
        ZMP (Zero Moment Point) is a method for making a walking robot keep balance. The goal of this implementation 
    is to deduce the trajectory of CoM with given ZMPs.
'''

import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are
import matplotlib.patches as patches

class WalkPattern3DGenerator(object):
    def __init__(self, sx = 0.3, sy = 0.1, N_step = 5, T_step = 1, dT = 5e-3, zc = 0.8, q = 1, r = 1e-6, g = 9.81, N = 320, step_angle = 0, step_directions = None):
        '''
            Parameters for generating ZMP patterns:
                sx: The step length in x-axis.
                sy: The step length in y-axis.
                N_step: Total number of steps (Taking one step by either left or right foot).
                T_step: Duration (seconds) of each step.
                dT: Sampling time (seconds).
                step_angle: Default step direction angle in radians (0 = forward, π/2 = left, π = backward, 3π/2 = right).
                step_directions: List of step directions for each step [angle, step_length, lateral_offset]. If None, uses default pattern.
                double_support_ratio: Ratio of double support phase duration to step duration (default: 0.2 or 20%)
        '''
        self.sx = sx
        self.sy = sy
        self.N_step = N_step
        self.T_step = T_step
        self.dT = dT
        self.step_angle = step_angle
        self.step_directions = step_directions
        
        # Walking phase parameters
        self.double_support_ratio = 0.2  # 20% of step time in double support phase
        self.swing_ratio = 0.8          # 80% of step time in swing phase
        
        # Foot placement constraints (in meters)
        self.min_foot_distance_x = 0.10  # Minimum horizontal distance between feet
        self.max_foot_distance_x = 0.30  # Maximum horizontal distance between feet
        self.min_foot_distance_y = 0.05  # Minimum lateral distance between feet
        self.max_foot_distance_y = 0.20  # Maximum lateral distance between feet
        
        # Adjust step parameters to satisfy constraints
        self.sy = max(self.sy, self.min_foot_distance_y)
        self.sy = min(self.sy, self.max_foot_distance_y)

        '''
            Parameters for the preview control system:
                dT: Sampling time (seconds).
                zc: The height of CoM.
                q, r: Positive weights.
                g: Gravity.
                N: Number of look-ahead samples in preview controller.
        '''
        self.zc = zc
        self.q = q
        self.r = r
        self.g = g
        self.N = N
        
        # Foot dimensions for visualization (in meters)
        self.foot_length = 0.1
        self.foot_width = 0.05
        
        # Store generated footsteps
        self.footsteps = None
        
    def generate_footsteps(self, sx, sy, N_step):
        '''
        Generate footstep positions with given parameters.
        
        Output:
            footsteps: List of dictionaries with footstep information:
                'position': (x, y) - position of the footstep
                'orientation': angle in radians
                'foot': 'left' or 'right' - which foot
        '''
        footsteps = []
        
        # Initial positions with proper foot distance
        # Place feet parallel to each other with proper distance
        foot_separation = max(self.min_foot_distance_y, min(sy, self.max_foot_distance_y))
        right_foot_pos = (0, -foot_separation/2)
        left_foot_pos = (0, foot_separation/2)
        current_angle = 0
        
        # Add initial stance
        footsteps.append({
            'position': right_foot_pos,
            'orientation': current_angle,
            'foot': 'right'
        })
        footsteps.append({
            'position': left_foot_pos,
            'orientation': current_angle,
            'foot': 'left'
        })
        
        # Start with left foot for first step
        is_left = True
        
        # Walk with omnidirectional capability
        xd, yd = 0, 0
        last_left_x, last_left_y = left_foot_pos
        last_right_x, last_right_y = right_foot_pos
        
        # Common reference point for the robot's center (between feet)
        center_x = 0
        center_y = 0
        
        for s in range(1, N_step + 1):
            if self.step_directions is not None and len(self.step_directions) >= s:
                # Use custom step directions
                angle, step_length, lateral_offset = self.step_directions[s-1]
                current_angle = angle
                
                # Determine lateral offset based on constraints
                lateral_offset = max(self.min_foot_distance_y/2, min(lateral_offset, self.max_foot_distance_y/2))
                
            else:
                # Use default pattern with specified angle
                angle = self.step_angle
                
                # Adjust step length to meet constraints
                step_length = max(self.min_foot_distance_x, min(sx, self.max_foot_distance_x))
                current_angle = angle
                
                # Determine lateral offset based on constraints
                lateral_offset = max(self.min_foot_distance_y/2, min(sy/2, self.max_foot_distance_y/2))
            
            # Calculate forward movement components
            dx = step_length * np.cos(angle)
            dy = step_length * np.sin(angle)
            
            # Update the robot's center position
            center_x += dx
            center_y += dy
            
            # Add lateral offset for foot placement
            if is_left:
                # Left foot position calculation
                foot_x = center_x + lateral_offset * np.sin(-angle)  # Lateral offset perpendicular to walking direction
                foot_y = center_y + lateral_offset * np.cos(angle)   # Lateral offset perpendicular to walking direction
                
                # Update last position
                last_left_x, last_left_y = foot_x, foot_y
            else:
                # Right foot position calculation
                foot_x = center_x - lateral_offset * np.sin(-angle)  # Lateral offset perpendicular to walking direction
                foot_y = center_y - lateral_offset * np.cos(angle)   # Lateral offset perpendicular to walking direction
                
                # Update last position
                last_right_x, last_right_y = foot_x, foot_y
            
            # Add the new footstep
            footsteps.append({
                'position': (foot_x, foot_y),
                'orientation': current_angle,
                'foot': 'left' if is_left else 'right'
            })
            
            is_left = not is_left  # Alternate feet
            
        return footsteps
    
    def create_ZMP_pattern(self, sx, sy, N_step, T_step, dT):
        '''
            Generate ZMP positions with given parameters.
            First generates footsteps, then creates ZMP trajectory.
        '''
        # Generate footsteps if not already generated
        if self.footsteps is None:
            self.footsteps = self.generate_footsteps(sx, sy, N_step)
            
        # Number of samples per step.
        N_sample = int(T_step / dT) 

        # Idle in the first and last step.
        patterns = np.empty([(N_step + 2) * N_sample, 2], dtype=np.float32)

        # Idle in the first step - ZMP at the center between feet
        initial_center_x = (self.footsteps[0]['position'][0] + self.footsteps[1]['position'][0]) / 2
        initial_center_y = (self.footsteps[0]['position'][1] + self.footsteps[1]['position'][1]) / 2
        patterns[:N_sample, :] = initial_center_x, initial_center_y

        # Generate ZMP trajectory from footsteps
        for s in range(1, N_step + 1):
            # Get the footstep position for this step
            footstep = self.footsteps[s + 1]  # +1 because we have the initial stance
            patterns[s * N_sample:(s + 1) * N_sample, :] = footstep['position']

        # Idle in the last step - use the final footstep position
        final_footstep = self.footsteps[-1]
        patterns[(N_step + 1) * N_sample:, :] = final_footstep['position']

        return patterns

    def create_omnidirectional_pattern(self, step_sequence):
        '''
            Create a custom omnidirectional walking pattern.
            Input:
                step_sequence: List of tuples [(angle, step_length, lateral_offset), ...]
                    angle: Direction angle in radians
                    step_length: Length of step in that direction
                    lateral_offset: Lateral offset for foot placement
        '''
        self.step_directions = step_sequence
        self.N_step = len(step_sequence)
        # Clear any previously generated footsteps
        self.footsteps = None
        return self.step_directions

    def set_walking_direction(self, angle):
        '''
            Set the walking direction angle.
            Input:
                angle: Walking direction in radians (0 = forward, π/2 = left, π = backward, 3π/2 = right)
        '''
        self.step_angle = angle

    def create_system(self, dT, zc, g):
        '''
            Output: 
                A, B, C: The matrices for the digital controller.
        '''
        A = np.array([
            [1, dT, dT**2/2],
            [0, 1, dT],
            [0, 0, 1]
        ])

        B = np.array([[dT**3/6, dT**2/2, dT]]).T

        C = np.array([[1, 0, -zc/g]])

        return A, B, C

    def create_controller(self, A, B, C, r, q, N):
        '''
            The controller is considered as a tracking control problem minimizing the performance
                J = \sum_j^\infty {Q (pref_j - p_j)**2 + R u_j**2}
            Output:
                K and Fs: For creating inputs that minimize J.
        '''
        # P: Solution of the Riccati equation.
        R = r * np.eye(1)
        Q = q * C.T @ C
        P = solve_discrete_are(A, B, Q, R)

        # Create K and Fs
        tmp = np.linalg.inv(R + B.T @ P @ B) @ B.T
        K = tmp @ P @ A
        
        Fs = []
        pre = np.copy(tmp)
        AcT = (A - B @ K).T
        for n in range(N):
            Fs.append(pre @ C.T * q)
            pre = pre @ AcT
        Fs = np.array([Fs]).flatten()

        return K, Fs

    def update_state(self, A, B, C, X, U):
        '''
            Output:
                X_next: The next state vector.
                P_k: The current ZMP position.
        '''
        X_next = A @ X + B @ U
        P_k = C @ X
        return X_next, P_k

    def solve_system(self, A, B, C, K, Fs, pref, N, x0, dx0, d2x0):
        '''
            Output:
                Xs: The state vector and ZMP in all sampling time.
                ZMPs: The prediction of ZMPs.
        '''
        # The initial state vector (all zeros by default).
        X = np.array([x0, dx0, d2x0]).T

        n_zmps = len(pref)
        pref_tmp = np.append(pref, [pref[-1]] * (N - 1))

        # Go over all samples.
        Xs, ZMPs = np.zeros(n_zmps), np.zeros(n_zmps)
        for i in range(n_zmps):
            U = -np.dot(K, X) + np.dot(Fs, pref_tmp[i:i + N])
            X, ZMP = self.update_state(A, B, C, X, U)
            Xs[i], ZMPs[i] = X[0], ZMP

        return Xs, ZMPs
    
    def generate(self, x0 = 0, dx0 = 0, d2x0 = 0, y0 = 0, dy0 = 0, d2y0 = 0):
        '''
            Input:
                x0, dx0, d2x0: The state vector [distance, velocity, acceleration] on x-axis.
                y0, dy0, d2y0: The state vector [distance, velocity, acceleration] on y-axis.
            Generate the trajectory of CoM that keeps the robot walking. The steps are:
                1. Create the parameter control system.
                2. Generate the footsteps.
                3. Generate the ZMPs from footsteps.
                4. Compute the trajectory of CoM and verify the ZMPs.
        '''
        # Create the controller.
        A, B, C = self.create_system(self.dT, self.zc, self.g)
        K, Fs = self.create_controller(A, B, C, self.r, self.q, self.N)

        # Generate the footsteps first
        self.footsteps = self.generate_footsteps(self.sx, self.sy, self.N_step)
        
        # Generate the ZMPs based on footsteps.
        pref = self.create_ZMP_pattern(self.sx, self.sy, self.N_step, self.T_step, self.dT)
        prefx, prefy = pref[:, 0], pref[:, 1]

        # With given ZMPs, compute the trajectory of CoM.
        Xs, Pxs = self.solve_system(A, B, C, K, Fs, pref[:, 0], self.N, x0, dx0, d2x0)
        Ys, Pys = self.solve_system(A, B, C, K, Fs, pref[:, 1], self.N, y0, dy0, d2y0)

        return Xs, Ys, Pxs, Pys, prefx, prefy
    
    def visualize_footprints_and_trajectory(self, Xs, Ys, Pxs, Pys, zmprefx, zmprefy):
        '''
        Visualize footprints along with ZMP and CoM trajectories.
        '''
        plt.figure(figsize=(12, 10))
        
        # Main plot - footprints with trajectories
        ax1 = plt.subplot(2, 1, 1)
        
        # Plot footprints
        if self.footsteps:
            left_feet = []
            right_feet = []
            
            for step in self.footsteps:
                x, y = step['position']
                angle = step['orientation']
                is_left = step['foot'] == 'left'
                
                # Create foot rectangle
                foot = patches.Rectangle(
                    (x - self.foot_length/2, y - self.foot_width/2),
                    self.foot_length, self.foot_width,
                    angle=np.degrees(angle),
                    color='blue' if is_left else 'red',
                    alpha=0.3,
                    label=f"{step['foot']} foot"
                )
                
                # Add foot to plot
                ax1.add_patch(foot)
                
                # Add text label for step number
                plt.text(x, y, f"{self.footsteps.index(step)}", 
                         ha='center', va='center', color='black', fontsize=8)
                
                # Collect foot positions for drawing connections
                if is_left:
                    left_feet.append((x, y))
                else:
                    right_feet.append((x, y))
            
            # Draw lines connecting consecutive left and right feet
            if len(left_feet) > 1:
                left_x, left_y = zip(*left_feet)
                plt.plot(left_x, left_y, 'b:', alpha=0.5, linewidth=1)
            
            if len(right_feet) > 1:
                right_x, right_y = zip(*right_feet)
                plt.plot(right_x, right_y, 'r:', alpha=0.5, linewidth=1)
        
        # Plot CoM and ZMP trajectories
        plt.plot(Xs, Ys, 'g-', linewidth=2, label='CoM')
        plt.plot(Pxs, Pys, 'b--', linewidth=1.5, label='Predicted ZMP')
        plt.plot(zmprefx, zmprefy, 'r:', linewidth=1.5, label='Reference ZMP')
        
        plt.title('Walking Pattern: Footprints, ZMP and CoM Trajectories')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.axis('equal')
        plt.grid(True)
        
        # Add legend with unique entries only
        handles, labels = plt.gca().get_legend_handles_labels()
        unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) 
                 if l not in [labels[j] for j in range(i)]]
        plt.legend(*zip(*unique))
        
        # X and Y trajectories over time
        ax2 = plt.subplot(2, 2, 3)
        plt.plot(range(len(Xs)), Xs, 'g-', label='CoM X')
        plt.plot(range(len(Pxs)), Pxs, 'b--', label='Pred ZMP x')
        plt.plot(range(len(zmprefx)), zmprefx, 'r:', label='Ref ZMP x')
        plt.legend()
        plt.title('X Position over Time')
        plt.xlabel('Time Steps')
        plt.ylabel('X [m]')
        plt.grid(True)
        
        ax3 = plt.subplot(2, 2, 4)
        plt.plot(range(len(Ys)), Ys, 'g-', label='CoM Y')
        plt.plot(range(len(Pys)), Pys, 'b--', label='Pred ZMP y')
        plt.plot(range(len(zmprefy)), zmprefy, 'r:', label='Ref ZMP y')
        plt.legend()
        plt.title('Y Position over Time')
        plt.xlabel('Time Steps')
        plt.ylabel('Y [m]')
        plt.grid(True)
        
        plt.tight_layout()

    def animate_walk(self, Xs, Ys, Pxs, Pys, zmprefx, zmprefy, save_animation=False, filename="walk_animation.mp4"):
        '''
        Create an animation showing the walking pattern with phases.
        
        Parameters:
            Xs, Ys: CoM trajectory
            Pxs, Pys: Predicted ZMP trajectory
            zmprefx, zmprefy: Reference ZMP trajectory
            save_animation: Whether to save the animation as a file
            filename: Name of the file to save animation to
        '''
        import matplotlib.animation as animation
        from matplotlib.patches import Rectangle, Circle
        
        # Create figure
        fig = plt.figure(figsize=(12, 8))
        ax = plt.subplot(1, 1, 1)
        
        # Set axis limits with some padding
        all_x = np.concatenate([Xs, Pxs, zmprefx])
        all_y = np.concatenate([Ys, Pys, zmprefy])
        x_min, x_max = min(all_x) - 0.5, max(all_x) + 0.5
        y_min, y_max = min(all_y) - 0.5, max(all_y) + 0.5
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        
        # Calculate time information
        N_total_samples = len(Xs)
        samples_per_step = int(self.T_step / self.dT)
        samples_double_support = int(samples_per_step * self.double_support_ratio)
        
        # Initialize visualization elements
        com_trail, = ax.plot([], [], 'g-', linewidth=2, label='CoM')
        zmp_trail, = ax.plot([], [], 'b--', linewidth=1.5, label='ZMP')
        ref_zmp_trail, = ax.plot([], [], 'r:', linewidth=1.5, label='Ref ZMP')
        
        # CoM marker
        com_marker = Circle((0, 0), radius=0.03, color='g', zorder=10)
        ax.add_patch(com_marker)
        
        # ZMP marker
        zmp_marker = Circle((0, 0), radius=0.02, color='b', zorder=9)
        ax.add_patch(zmp_marker)
        
        # Initial feet - properly positioned based on initial footsteps
        if self.footsteps and len(self.footsteps) >= 2:
            # Get initial foot positions from footsteps
            right_pos = self.footsteps[0]['position']
            left_pos = self.footsteps[1]['position']
            right_angle = self.footsteps[0]['orientation']
            left_angle = self.footsteps[1]['orientation']
            
            # Create properly positioned foot rectangles
            left_foot = Rectangle(
                (left_pos[0] - self.foot_length/2, left_pos[1] - self.foot_width/2),
                self.foot_length, self.foot_width, 
                angle=np.degrees(left_angle),
                color='blue', alpha=0.5, zorder=5
            )
            right_foot = Rectangle(
                (right_pos[0] - self.foot_length/2, right_pos[1] - self.foot_width/2),
                self.foot_length, self.foot_width, 
                angle=np.degrees(right_angle),
                color='red', alpha=0.5, zorder=5
            )
        else:
            # Fallback if no footsteps are available
            foot_separation = self.sy
            left_foot = Rectangle(
                (-self.foot_length/2, foot_separation/2 - self.foot_width/2),
                self.foot_length, self.foot_width, 
                color='blue', alpha=0.5, zorder=5
            )
            right_foot = Rectangle(
                (-self.foot_length/2, -foot_separation/2 - self.foot_width/2),
                self.foot_length, self.foot_width, 
                color='red', alpha=0.5, zorder=5
            )
            
        ax.add_patch(left_foot)
        ax.add_patch(right_foot)
        
        # Future footprints (semi-transparent)
        future_feet = []
        if self.footsteps:
            for i, step in enumerate(self.footsteps):
                if i < 2:  # Skip initial stance feet (already shown as active)
                    continue
                x, y = step['position']
                angle = step['orientation']
                is_left = step['foot'] == 'left'
                
                foot = Rectangle(
                    (x - self.foot_length/2, y - self.foot_width/2),
                    self.foot_length, self.foot_width,
                    angle=np.degrees(angle),
                    color='blue' if is_left else 'red',
                    alpha=0.2,
                    zorder=1
                )
                ax.add_patch(foot)
                future_feet.append((foot, i))
        
        # Phase text display
        phase_text = ax.text(x_min + 0.1, y_max - 0.1, "Initializing", fontsize=12, 
                            bbox=dict(facecolor='white', alpha=0.7))
        
        # Time text display
        time_text = ax.text(x_min + 0.1, y_max - 0.2, "Time: 0.0s", fontsize=10,
                           bbox=dict(facecolor='white', alpha=0.7))
        
        # Add legend
        ax.legend(loc='upper right')
        ax.set_title('Walking Pattern Animation with Phases')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.grid(True)
        
        def update_plot(frame):
            # Get the current position data
            if frame < len(Xs):
                com_x, com_y = Xs[frame], Ys[frame]
                zmp_x, zmp_y = Pxs[frame], Pys[frame]
                
                # Update trails
                com_trail.set_data(Xs[:frame+1], Ys[:frame+1])
                zmp_trail.set_data(Pxs[:frame+1], Pys[:frame+1])
                ref_zmp_trail.set_data(zmprefx[:frame+1], zmprefy[:frame+1])
                
                # Update markers
                com_marker.center = (com_x, com_y)
                zmp_marker.center = (zmp_x, zmp_y)
                
                # Calculate current step and phase
                step_index = frame // samples_per_step  # Current step (0 = initial)
                step_phase = frame % samples_per_step   # Phase within step
                
                # Determine if we're in double or single support phase
                in_double_support = step_phase < samples_double_support
                
                # Real-time in seconds
                current_time = frame * self.dT
                time_text.set_text(f"Time: {current_time:.2f}s")
                
                # Update foot positions and phases
                if step_index < self.N_step + 1:  # +1 for initial stance
                    # Set phase text
                    if step_index == 0:
                        phase_text.set_text("Initial Double Support")
                    elif in_double_support:
                        phase_text.set_text("Double Support Phase")
                    else:
                        # Determine which foot is swinging
                        is_left_swing = (step_index % 2) == 1
                        swing_foot = "Left" if is_left_swing else "Right"
                        phase_text.set_text(f"{swing_foot} Foot Swing Phase")
                    
                    # Update foot visuals
                    if step_index > 0:
                        # Get the correct foot positions
                        current_step = min(step_index + 1, len(self.footsteps) - 1)  # Current active step
                        prev_step = max(0, current_step - 1)  # Previous step
                        
                        # Check which foot is currently in contact (stance foot)
                        is_left_stance = (step_index % 2) == 0
                        
                        # Get current and previous footstep positions
                        if is_left_stance:
                            # Left foot is stance
                            stance_foot_pos = self.footsteps[current_step-1]['position'] if current_step > 0 else (0, self.sy/2)
                            stance_foot_angle = self.footsteps[current_step-1]['orientation'] if current_step > 0 else 0
                            
                            # Right foot is either in swing or in previous position
                            if in_double_support:
                                # Both feet on ground
                                swing_foot_pos = self.footsteps[current_step-2]['position'] if current_step > 1 else (0, -self.sy/2)
                                swing_foot_angle = self.footsteps[current_step-2]['orientation'] if current_step > 1 else 0
                                right_opacity = 0.5  # Fully visible during double support
                            else:
                                # Right foot is swinging - interpolate position
                                swing_phase_ratio = (step_phase - samples_double_support) / (samples_per_step - samples_double_support)
                                
                                # Previous position
                                prev_pos = self.footsteps[current_step-2]['position'] if current_step > 1 else (0, -self.sy/2)
                                prev_angle = self.footsteps[current_step-2]['orientation'] if current_step > 1 else 0
                                
                                # Target position
                                target_pos = self.footsteps[current_step]['position']
                                target_angle = self.footsteps[current_step]['orientation']
                                
                                # Add a simple arc to the foot trajectory during swing
                                swing_height = 0.1  # Maximum height of swing in meters
                                height_factor = swing_phase_ratio * (1 - swing_phase_ratio) * 4  # Parabolic curve peaking at 0.5
                                
                                # Linear interpolation with height
                                swing_foot_pos = (
                                    prev_pos[0] + swing_phase_ratio * (target_pos[0] - prev_pos[0]),
                                    prev_pos[1] + swing_phase_ratio * (target_pos[1] - prev_pos[1]),
                                )
                                
                                swing_foot_angle = prev_angle + swing_phase_ratio * (target_angle - prev_angle)
                                right_opacity = 0.3  # Semi-transparent during swing
                            
                            # Update foot representations
                            left_foot.set_alpha(0.5)
                            left_foot.set_xy((stance_foot_pos[0] - self.foot_length/2, stance_foot_pos[1] - self.foot_width/2))
                            left_foot.angle = np.degrees(stance_foot_angle)
                            
                            right_foot.set_alpha(right_opacity)
                            right_foot.set_xy((swing_foot_pos[0] - self.foot_length/2, swing_foot_pos[1] - self.foot_width/2))
                            right_foot.angle = np.degrees(swing_foot_angle)
                            
                        else:
                            # Right foot is stance
                            stance_foot_pos = self.footsteps[current_step-1]['position'] if current_step > 0 else (0, -self.sy/2)
                            stance_foot_angle = self.footsteps[current_step-1]['orientation'] if current_step > 0 else 0
                            
                            # Left foot is either in swing or in previous position
                            if in_double_support:
                                # Both feet on ground
                                swing_foot_pos = self.footsteps[current_step-2]['position'] if current_step > 1 else (0, self.sy/2)
                                swing_foot_angle = self.footsteps[current_step-2]['orientation'] if current_step > 1 else 0
                                left_opacity = 0.5  # Fully visible during double support
                            else:
                                # Left foot is swinging - interpolate position
                                swing_phase_ratio = (step_phase - samples_double_support) / (samples_per_step - samples_double_support)
                                
                                # Previous position
                                prev_pos = self.footsteps[current_step-2]['position'] if current_step > 1 else (0, self.sy/2)
                                prev_angle = self.footsteps[current_step-2]['orientation'] if current_step > 1 else 0
                                
                                # Target position
                                target_pos = self.footsteps[current_step]['position']
                                target_angle = self.footsteps[current_step]['orientation']
                                
                                # Add a simple arc to the foot trajectory during swing
                                swing_height = 0.1  # Maximum height of swing in meters
                                height_factor = swing_phase_ratio * (1 - swing_phase_ratio) * 4  # Parabolic curve peaking at 0.5
                                
                                # Linear interpolation with height
                                swing_foot_pos = (
                                    prev_pos[0] + swing_phase_ratio * (target_pos[0] - prev_pos[0]),
                                    prev_pos[1] + swing_phase_ratio * (target_pos[1] - prev_pos[1]),
                                )
                                
                                swing_foot_angle = prev_angle + swing_phase_ratio * (target_angle - prev_angle)
                                left_opacity = 0.3  # Semi-transparent during swing
                            
                            # Update foot representations
                            right_foot.set_alpha(0.5)
                            right_foot.set_xy((stance_foot_pos[0] - self.foot_length/2, stance_foot_pos[1] - self.foot_width/2))
                            right_foot.angle = np.degrees(stance_foot_angle)
                            
                            left_foot.set_alpha(left_opacity)
                            left_foot.set_xy((swing_foot_pos[0] - self.foot_length/2, swing_foot_pos[1] - self.foot_width/2))
                            left_foot.angle = np.degrees(swing_foot_angle)
                
            return [com_trail, zmp_trail, ref_zmp_trail, com_marker, zmp_marker, 
                    left_foot, right_foot, phase_text, time_text]
        
        # Calculate appropriate frame interval for real-time playback
        # We'll use a factor to slow down or speed up the animation
        speed_factor = 0.5  # 0.5 means half speed (slower)
        interval = self.dT * 1000 * speed_factor  # Convert to milliseconds
        
        # Create animation
        anim = animation.FuncAnimation(
            fig, update_plot, frames=min(N_total_samples, 1000),  # Limit frames for performance
            interval=interval, blit=True
        )
        
        # Save animation if requested
        if save_animation:
            writer = animation.FFMpegWriter(fps=30)
            anim.save(filename, writer=writer)
            print(f"Animation saved as {filename}")
        
        plt.tight_layout()
        plt.show()
        
        return anim


if __name__ == '__main__':
    # Example 1: Forward walking (default)
    generator = WalkPattern3DGenerator(sx=0.2, sy=0.15, T_step=0.8)  # Adjusted step time
    Xs, Ys, Pxs, Pys, zmprefx, zmprefy = generator.generate()

    # Animate with phases
    anim1 = generator.animate_walk(Xs, Ys, Pxs, Pys, zmprefx, zmprefy)

    # Show static visualization as well
    generator.visualize_footprints_and_trajectory(Xs, Ys, Pxs, Pys, zmprefx, zmprefy)
    
    # Example 2: Diagonal walking
    generator_diag = WalkPattern3DGenerator(sx=0.2, sy=0.15, step_angle=np.pi/4, T_step=0.8)  # 45 degrees
    Xs_diag, Ys_diag, Pxs_diag, Pys_diag, zmprefx_diag, zmprefy_diag = generator_diag.generate()

    # Fixed: Use the correct generator instance for animation
    anim2 = generator_diag.animate_walk(Xs_diag, Ys_diag, Pxs_diag, Pys_diag, zmprefx_diag, zmprefy_diag)
    
    # Show static visualization as well
    generator_diag.visualize_footprints_and_trajectory(Xs_diag, Ys_diag, Pxs_diag, Pys_diag, zmprefx_diag, zmprefy_diag)
    
    # Example 3: Complex pattern - make a square
    square_pattern = [
        (0, 0.2, 0.15),           # Forward
        (np.pi/2, 0.2, 0.15),     # Left
        (np.pi, 0.2, 0.15),       # Backward
        (3*np.pi/2, 0.2, 0.15),   # Right
    ]
    
    generator_complex = WalkPattern3DGenerator(N_step=4, T_step=0.8)
    generator_complex.create_omnidirectional_pattern(square_pattern)
    Xs_complex, Ys_complex, Pxs_complex, Pys_complex, zmprefx_complex, zmprefy_complex = generator_complex.generate()
    
    # Animate complex pattern
    anim3 = generator_complex.animate_walk(Xs_complex, Ys_complex, Pxs_complex, Pys_complex, 
                                         zmprefx_complex, zmprefy_complex, 
                                         save_animation=True, filename="square_walk.mp4")