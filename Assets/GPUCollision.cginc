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

	// 計算 line 與 sphere 是否碰撞
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
	// 兩個解
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
