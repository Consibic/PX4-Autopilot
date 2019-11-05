/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ControlAllocationMultirotor.hpp
 *
 * Control Allocation Algorithm for multirotors
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 * @author Roman Bapst <roman@auterion.com>
 */

#include "ControlAllocationMultirotor.hpp"



void
ControlAllocationMultirotor::setEffectivenessMatrix(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &B)
{
	_B = B;
	_A = matrix::geninv(_B);
}


void
ControlAllocationMultirotor::allocate()
{
	// Allocate
	_actuator_sp = _A * _control_sp;

	// go through control axes from lowest to highest priority and unsaturate the actuators
	for (unsigned i = 0; i < NUM_AXES; i++) {
		_actuator_sp = desaturateActuators(_actuator_sp, _axis_prio_increasing[i]);
	}

	// Clip
	_actuator_sp = clipActuatorSetpoint();

	// Compute achieved control
	_control_allocated = _B * _actuator_sp;
}

ControlAllocation::ActuatorVector ControlAllocationMultirotor::desaturateActuators(ActuatorVector actuator_sp,
		ControlAxis axis)
{
	ActuatorVector desaturation_vector = getDesaturationVector(axis);

	float gain = computeDesaturationGain(desaturation_vector, actuator_sp);

	actuator_sp = actuator_sp + gain * desaturation_vector;

	gain = computeDesaturationGain(desaturation_vector, actuator_sp);

	actuator_sp = actuator_sp + 0.5f * gain * desaturation_vector;

	return actuator_sp;

}

ControlAllocation::ActuatorVector ControlAllocationMultirotor::getDesaturationVector(ControlAxis axis)
{
	ActuatorVector ret;

	for (unsigned i = 0; i < NUM_ACTUATORS; i++) {
		ret(i) = _A(i, axis);
	}

	return ret;
}


float ControlAllocationMultirotor::computeDesaturationGain(ActuatorVector desaturation_vector,
		ActuatorVector actuator_sp)
{
	float k_min = 0.f;
	float k_max = 0.f;

	for (unsigned i = 0; i < NUM_ACTUATORS; i++) {
		// Avoid division by zero. If desaturation_vector(i) is zero, there's nothing we can do to unsaturate anyway
		if (fabsf(desaturation_vector(i)) < FLT_EPSILON) {
			continue;
		}

		if (actuator_sp(i) < _actuator_min(i)) {
			float k = (_actuator_min(i) - actuator_sp(i)) / desaturation_vector(i);

			if (k < k_min) { k_min = k; }

			if (k > k_max) { k_max = k; }
		}

		if (actuator_sp(i) > _actuator_max(i)) {
			float k = (_actuator_max(i) - actuator_sp(i)) / desaturation_vector(i);

			if (k < k_min) { k_min = k; }

			if (k > k_max) { k_max = k; }
		}
	}

	// Reduce the saturation as much as possible
	return k_min + k_max;
}
