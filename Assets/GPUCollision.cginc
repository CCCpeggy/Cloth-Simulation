//
// DEFINES
//

#define EPSILON 0.001



//
// TYPES
//

struct Ray {
	float3 origin, direction;
};

struct SphereCollider {
	float3 center;
	float radius;
};

struct BoxCollider {
	float3 center;
	float3 extents;
};

struct Hit {
	bool collision;
	float3 hitPoint,  hitNormal;
};


//
// UTILITY METHODS
//

float3 HitPoint(Ray r, SphereCollider s) {
	// If the ray is sufficiently short, just return the midpoint.
	if (length(r.direction) < EPSILON) {
		return r.origin +r.direction/2.0;
	}
	float3 P = r.origin;
	float3 C = s.center;
	float R = s.radius;
	float3 d = r.direction;

	float a = dot(d, (P - C));
	float b = a * a - dot(P - C, P - C) + R * R;
	if(b < 0) return float3 (0, 0, 0);
	float t = (-a - sqrt(b)) / length(r.direction) / length(r.direction) ;
	return r.origin + t*normalize(r.direction);
	// return r.origin + (dot(p - r.origin, r.direction) / length(r.direction));
}

float3 Reflect(float3 original, float3 normal) {
	return original - 2.0*dot(original, normal)*normal;
}



//
// COLLISIONS
//

/* Ray Sphere Intersection. */
Hit RaySphereCollision(Ray r, SphereCollider s) {
	Hit h;
	h.collision = false;
	h.hitPoint = float3(0,0,0);
	h.hitNormal = float3(0,1,0);

	float3 P = r.origin;
	float3 C = s.center;
	float R = s.radius;
	float3 d = r.direction;
	float dLength = length(d);

	float3 oc = s.center - r.origin;
    float projoc = dot(r.direction, oc);

    if (projoc < 0)
        return h;

    float oc2 = dot(oc, oc);
    float distance2 = oc2 - projoc * projoc;

    if (distance2 > s.radius)
        return h;

    float discriminant = s.radius - distance2;
	discriminant = sqrt(discriminant);
	float t0 = projoc - discriminant;
	float t1 = projoc + discriminant;
	float t;
	if (t0 >= 0 && t0 <= 1) {
		t = t0;
	}
	else if (t1 >= 0 && t1 <= 1) {
		t = t1;
	}
	else {
		return h;
	}
	float3 hitPoint = P + t * d;

	
	h.collision = true;
	h.hitNormal = normalize(hitPoint - C);
	h.hitPoint = hitPoint + h.hitNormal * 0.01;

	return h;
}

Hit RayBoxCollision(Ray r, BoxCollider b) {
	Hit h;
	h.collision = false;
	h.hitPoint = float3(0, 0, 0);
	h.hitNormal = h.hitPoint * h.hitNormal;
	return h;
}
