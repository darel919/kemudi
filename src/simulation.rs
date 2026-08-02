//! Fixed-step simulation orchestration.
//!
//! Vehicle systems, integration, constraints, and collision/damage live in
//! focused submodules while their inherent PhysicsWorld methods remain at the
//! same crate-visible API paths.

mod collision;
mod constraints;
mod integration;
mod vehicle_forces;

use crate::types::PhysicsWorld;

impl PhysicsWorld {
    pub(crate) fn step_fixed(&mut self) {
        if self.constraint_start_positions.len() != self.nodes.len() {
            self.constraint_start_positions
                .resize(self.nodes.len(), [0.0; 3]);
        }
        self.update_mass_properties();
        self.apply_vehicle_forces();
        self.apply_forces();
        self.integrate_velocities(self.fixed_dt);
        // Save the unconstrained predicted positions. Constraint projection
        // is a position correction; retaining this state lets us add only the
        // correction back into velocity without discarding force and damping
        // integration from the current step.
        for (start, node) in self.constraint_start_positions.iter_mut().zip(&self.nodes) {
            *start = [node.x, node.y, node.z];
        }
        self.solve_xpbd_constraints();
        self.collide_with_terrain();
        self.apply_drag();
        self.update_telemetry();
    }
}
