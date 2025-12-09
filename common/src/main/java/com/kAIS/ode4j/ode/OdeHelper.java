/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Open Dynamics Engine 4J, Copyright (C) 2009-2014 Tilmann Zaeschke     *
 * All rights reserved.  Email: ode4j@gmx.de   Web: www.ode4j.org        *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT.         *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT, ODE-LICENSE-BSD.TXT and ODE4J-LICENSE-BSD.TXT for more   *
 * details.                                                              *
 *                                                                       *
 *************************************************************************/
package com.kAIS.ode4j.ode;

import java.io.File;
import java.util.List;

import com.kAIS.ode4j.math.DVector3;
import com.kAIS.ode4j.math.DVector3C;
import com.kAIS.ode4j.ode.DGeom.DNearCallback;
import com.kAIS.ode4j.ode.DTriMesh.DTriCallback;
import com.kAIS.ode4j.ode.DTriMesh.DTriRayCallback;
import com.kAIS.ode4j.ode.internal.*;
import com.kAIS.ode4j.ode.internal.joints.DxJointConstrainedBall;
import com.kAIS.ode4j.ode.internal.joints.DxJointGroup;
import com.kAIS.ode4j.ode.internal.joints.OdeJointsFactoryImpl;
import com.kAIS.ode4j.ode.internal.ragdoll.DxRagdoll;
import com.kAIS.ode4j.ode.internal.trimesh.DxTriMesh;
import com.kAIS.ode4j.ode.internal.trimesh.DxTriMeshData;
import com.kAIS.ode4j.ode.internal.OdeFactoryImpl;
import com.kAIS.ode4j.ode.ragdoll.DRagdoll;
import com.kAIS.ode4j.ode.ragdoll.DRagdollConfig;

/**
 * This is the general helper class for ode4j.
 * <p>
 * It provides:
 * <ul>
 * <li> Factory methods for most of the classes in ode4j </li>
 * <li> Initialisation methods ({@code initOde2()} </li>
 * <li> Collision methods </li>
 * <li> Other helper methods </li>
 * </ul>
 */
public abstract class OdeHelper {

	/** 내부 구현 팩토리 (Dx* 를 생성하고, public D* 인터페이스를 돌려줌) */
	private static final OdeFactoryImpl ODE = new OdeFactoryImpl();

	//****************************************************************************
	// joints

	/**
	 * Create a joint group.
	 * @return joint group
	 */
	public static DJointGroup createJointGroup () {
		// param max_size deprecated. Set to -1.
		return DxJointGroup.dJointGroupCreate(-1);
	}

	/**
	 * Create a new joint feedback.
	 * @return joint feedback
	 */
	public static DJoint.DJointFeedback createJointFeedback() {
		return new DJoint.DJointFeedback();
	}

	// AMotorJoint
	public static DAMotorJoint createAMotorJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateAMotor(world, group);
	}
	public static DAMotorJoint createAMotorJoint (DWorld world) {
		return ODE.dJointCreateAMotor(world, null);
	}

	// BallJoint
	public static DBallJoint createBallJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateBall(world, group);
	}
	public static DBallJoint createBallJoint (DWorld world) {
		return ODE.dJointCreateBall(world, null);
	}

	// ConstrainedBallJoint (ode4j 확장, 내부 타입 반환)
	public static DxJointConstrainedBall createConstrainedBallJoint (DWorld world, DJointGroup group) {
		// OdeFactoryImpl 에 전용 메서드가 없으니, 내부 조인트 팩토리 직접 사용
		OdeJointsFactoryImpl jf = new OdeJointsFactoryImpl();
		return jf.dJointCreateConstrainedBall(world, group);
	}
	public static DxJointConstrainedBall createConstrainedBallJoint (DWorld world) {
		OdeJointsFactoryImpl jf = new OdeJointsFactoryImpl();
		return jf.dJointCreateConstrainedBall(world, null);
	}

	// ContactJoint
	public static DContactJoint createContactJoint (DWorld world, DJointGroup group, DContact c) {
		return ODE.dJointCreateContact(world, group, c);
	}
	public static DContactJoint createContactJoint (DWorld world, DContact c) {
		return ODE.dJointCreateContact(world, null, c);
	}

	// Double Ball
	public static DDoubleBallJoint createDBallJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateDBall(world, group);
	}
	public static DDoubleBallJoint createDBallJoint (DWorld world) {
		return ODE.dJointCreateDBall(world, null);
	}

	// Double Hinge
	public static DDoubleHingeJoint createDHingeJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateDHinge(world, group);
	}
	public static DDoubleHingeJoint createDHingeJoint (DWorld world) {
		return ODE.dJointCreateDHinge(world, null);
	}

	// FixedJoint
	public static DFixedJoint createFixedJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateFixed(world, group);
	}
	public static DFixedJoint createFixedJoint (DWorld world) {
		return ODE.dJointCreateFixed(world, null);
	}

	// HingeJoint
	public static DHingeJoint createHingeJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateHinge(world, group);
	}
	public static DHingeJoint createHingeJoint (DWorld world) {
		return ODE.dJointCreateHinge(world, null);
	}

	// Hinge2Joint
	public static DHinge2Joint createHinge2Joint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateHinge2(world, group);
	}
	public static DHinge2Joint createHinge2Joint (DWorld world) {
		return ODE.dJointCreateHinge2(world, null);
	}

	// LMotorJoint
	public static DLMotorJoint createLMotorJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateLMotor(world, group);
	}
	public static DLMotorJoint createLMotorJoint (DWorld world) {
		return ODE.dJointCreateLMotor(world, null);
	}

	// NullJoint
	public static DNullJoint createNullJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateNull(world, group);
	}
	public static DNullJoint createNullJoint (DWorld world) {
		return ODE.dJointCreateNull(world, null);
	}

	// PistonJoint
	public static DPistonJoint createPistonJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreatePiston(world, group);
	}
	public static DPistonJoint createPistonJoint (DWorld world) {
		return ODE.dJointCreatePiston(world, null);
	}

	// Plane2DJoint
	public static DPlane2DJoint createPlane2DJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreatePlane2D(world, group);
	}
	public static DPlane2DJoint createPlane2DJoint (DWorld world) {
		return ODE.dJointCreatePlane2D(world, null);
	}

	// PRJoint
	public static DPRJoint createPRJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreatePR(world, group);
	}
	public static DPRJoint createPRJoint (DWorld world) {
		return ODE.dJointCreatePR(world, null);
	}

	// PUJoint
	public static DPUJoint createPUJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreatePU(world, group);
	}
	public static DPUJoint createPUJoint (DWorld world) {
		return ODE.dJointCreatePU(world, null);
	}

	// SliderJoint
	public static DSliderJoint createSliderJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateSlider(world, group);
	}
	public static DSliderJoint createSliderJoint (DWorld world) {
		return ODE.dJointCreateSlider(world, null);
	}

	// TransmissionJoint
	public static DTransmissionJoint createTransmissionJoint (DWorld w, DJointGroup group) {
		return ODE.dJointCreateTransmission(w, group);
	}
	public static DTransmissionJoint createTransmissionJoint (DWorld w) {
		return ODE.dJointCreateTransmission(w, null);
	}

	// UniversalJoint
	public static DUniversalJoint createUniversalJoint (DWorld world, DJointGroup group) {
		return ODE.dJointCreateUniversal(world, group);
	}
	public static DUniversalJoint createUniversalJoint (DWorld world) {
		return ODE.dJointCreateUniversal(world, null);
	}

	//****************************************************************************
	// World / Bodies / Spaces / Geoms

	public static DWorld createWorld () {
		return DxWorld.dWorldCreate();
	}

	public static DMass createMass() {
		return new DxMass();
	}

	public static DBody createBody (DWorld w){
		return DxBody.dBodyCreate((DxWorld) w);
	}

	public static DSimpleSpace createSimpleSpace () {
		return DxSimpleSpace.dSimpleSpaceCreate(null);
	}
	public static DSimpleSpace createSimpleSpace (DSpace space) {
		return DxSimpleSpace.dSimpleSpaceCreate((DxSpace) space);
	}

	public static DSapSpace createSapSpace (DSapSpace.AXES axes) {
		return DxSAPSpace.dSweepAndPruneSpaceCreate(null, axes.getCode());
	}
	public static DSapSpace createSapSpace (DSpace space, DSapSpace.AXES axes) {
		return DxSAPSpace.dSweepAndPruneSpaceCreate((DxSpace) space, axes.getCode());
	}

	public static DSapSpace createSapSpace2 (DSapSpace.AXES axes, long staticGeomCategoryMask) {
		return DxSAPSpace2.dSweepAndPruneSpaceCreate(null, axes.getCode(), staticGeomCategoryMask);
	}
	public static DSapSpace createSapSpace2 (DSpace space, DSapSpace.AXES axes, long staticGeomCategoryMask) {
		return DxSAPSpace2.dSweepAndPruneSpaceCreate((DxSpace) space, axes.getCode(), staticGeomCategoryMask);
	}

	public static DHashSpace createHashSpace () {
		return DxHashSpace.dHashSpaceCreate(null);
	}
	public static DHashSpace createHashSpace (DSpace space) {
		return DxHashSpace.dHashSpaceCreate((DxSpace)space);
	}

	public static DQuadTreeSpace createQuadTreeSpace (DVector3C Center, DVector3C Extents, int Depth) {
		return DxQuadTreeSpace.dQuadTreeSpaceCreate(null, Center, Extents, Depth);
	}
	public static DQuadTreeSpace createQuadTreeSpace (DSpace space,
													  DVector3C Center, DVector3C Extents, int Depth) {
		return DxQuadTreeSpace.dQuadTreeSpaceCreate((DxSpace) space, Center, Extents, Depth);
	}

	@Deprecated
	public static DBhvSpace createBHVSpace (long staticGeomCategoryMask) {
		return DxBVHSpace.bvhSpaceCreate(null, 16, false, 0.2, staticGeomCategoryMask);
	}

	@Deprecated
	public static DBhvSpace createBHVSpace (DSpace space, int nodesPerLeaf, boolean highQuality,
											double fatAabbMargin, long staticGeomCategoryMask) {
		return DxBVHSpace.bvhSpaceCreate((DxSpace) space, nodesPerLeaf, highQuality,
				fatAabbMargin, staticGeomCategoryMask);
	}

	public static DBVHSpace createBVHSpace (long staticGeomCategoryMask) {
		return DxBVHSpace.bvhSpaceCreate(null, 16, false, 0.2, staticGeomCategoryMask);
	}
	public static DBVHSpace createBVHSpace (DSpace space, int nodesPerLeaf, boolean highQuality,
											double fatAabbMargin, long staticGeomCategoryMask) {
		return DxBVHSpace.bvhSpaceCreate((DxSpace) space, nodesPerLeaf, highQuality,
				fatAabbMargin, staticGeomCategoryMask);
	}

	// Box
	public static DBox createBox(double lx, double ly, double lz) {
		return DxBox.dCreateBox(null, lx, ly, lz);
	}
	public static DBox createBox(DVector3 lxyz) {
		return DxBox.dCreateBox(null, lxyz.get0(), lxyz.get1(), lxyz.get2());
	}
	public static DBox createBox(DSpace space, double lx, double ly, double lz) {
		return DxBox.dCreateBox((DxSpace) space, lx, ly, lz);
	}
	public static DBox createBox(DSpace space, DVector3 lxyz) {
		return DxBox.dCreateBox((DxSpace) space, lxyz.get0(), lxyz.get1(), lxyz.get2());
	}

	// Capsule
	public static DCapsule createCapsule(double radius, double length) {
		return DxCapsule.dCreateCapsule(null, radius, length);
	}
	public static DCapsule createCapsule(DSpace space, double radius, double length) {
		return DxCapsule.dCreateCapsule((DxSpace) space, radius, length);
	}

	// Convex
	public static DConvex createConvex(double[] planes, double[] points, int[] polygons) {
		return DxConvex.dCreateConvex(null, planes, points, polygons);
	}
	public static DConvex createConvex(DSpace space, double[] planes, double[] points, int[] polygons) {
		return DxConvex.dCreateConvex((DxSpace)space, planes, points, polygons);
	}
	public static DConvex createConvex(double[] planes,
									   int planeCount, double[] points, int pointCount, int[] polygons) {
		return DxConvex.dCreateConvex(null, planes, planeCount, points, pointCount, polygons);
	}
	public static DConvex createConvex(DSpace space, double[] planes,
									   int planeCount, double[] points, int pointCount, int[] polygons) {
		return DxConvex.dCreateConvex((DxSpace)space, planes, planeCount, points, pointCount, polygons);
	}

	// Cylinder
	public static DCylinder createCylinder(double radius, double length) {
		return DxCylinder.dCreateCylinder(null, radius, length);
	}
	public static DCylinder createCylinder(DSpace space, double radius, double length) {
		return DxCylinder.dCreateCylinder((DxSpace)space, radius, length);
	}

	// Plane
	public static DPlane createPlane (DSpace space, double a, double b, double c, double d) {
		return DxPlane.dCreatePlane((DxSpace) space, a, b, c, d);
	}

	// Ragdoll
	public static DRagdoll createRagdoll(DWorld world, DRagdollConfig skeleton) {
		return DxRagdoll.create(world, null, skeleton);
	}
	public static DRagdoll createRagdoll(DWorld world, DSpace space, DRagdollConfig skeleton) {
		return DxRagdoll.create(world, space, skeleton);
	}

	// Ray
	public static DRay createRay(int length) {
		return DxRay.dCreateRay(null, length);
	}
	public static DRay createRay(DSpace space, double length) {
		return DxRay.dCreateRay((DxSpace) space, length);
	}

	// Sphere
	public static DSphere createSphere(double radius) {
		return DxSphere.dCreateSphere(null, radius);
	}
	public static DSphere createSphere(DSpace space, double radius) {
		return DxSphere.dCreateSphere((DxSpace)space, radius);
	}

	//****************************************************************************
	// ODE init / shutdown

	public static void initODE() {
		OdeInit.dInitODE();
	}

	public static int initODE2(int uiInitFlags/*=0*/) {
		return OdeInit.dInitODE2(uiInitFlags) ? 1 : 0;
	}

	public static void closeODE() {
		OdeInit.dCloseODE();
	}

	//****************************************************************************
	// Collision helpers

	public static int collide (DGeom o1, DGeom o2, int flags,
							   DContactGeomBuffer contacts) {
		return DxGeom.dCollide((DxGeom)o1, (DxGeom)o2, flags, contacts, 1);
	}

	public static void spaceCollide (DSpace space, Object data, DNearCallback callback) {
		((DxSpace)space).dSpaceCollide(data, callback);
	}

	public static void spaceCollide2(DGeom space1, DGeom space2, Object data,
									 DNearCallback callback) {
		DxSpace.dSpaceCollide2((DxGeom)space1, (DxGeom)space2, data, callback);
	}

	public static void setColliderOverride (int i, int j, DColliderFn fn) {
		DxGeom.dSetColliderOverride(i, j, fn);
	}

	//****************************************************************************
	// Connection helpers

	public static boolean areConnected (DBody b1, DBody b2) {
		return ODE._dAreConnected(b1, b2);
	}

	@SafeVarargs
	public static boolean areConnectedExcluding (DBody body1, DBody body2,
												 Class<? extends DJoint> ... jointTypes) {
		return ODE._dAreConnectedExcluding(body1, body2, jointTypes);
	}

	@SuppressWarnings("unchecked")
	public static boolean areConnectedExcluding (DBody body1, DBody body2,
												 Class<? extends DJoint> jointType) {
		return ODE._dAreConnectedExcluding(body1, body2, jointType);
	}

	public static DJoint connectingJoint(DBody b1, DBody b2) {
		return OdeJointsFactoryImpl.dConnectingJoint(b1, b2);
	}

	public static List<DJoint> connectingJointList(DBody b1, DBody b2) {
		return OdeJointsFactoryImpl.dConnectingJointList((DxBody)b1, (DxBody)b2);
	}

	//****************************************************************************
	// Configuration & version

	public static boolean checkConfiguration( String extension ) {
		return ODE._dCheckConfiguration(extension);
	}

	public static String getConfiguration () {
		return ODE._dGetConfiguration();
	}

	public static String getVersion() {
		return "0.5.2";
	}

	@Deprecated
	public static void worldExportDIF(DWorld world, File f, String string) {
		throw new UnsupportedOperationException(); //TODO
	}

	@Deprecated
	public static int allocateODEDataForThread(int uiAllocateFlags) {
		return OdeInit.dAllocateODEDataForThread(uiAllocateFlags) ? 1 : 0;
	}

	//****************************************************************************
	// Heightfield / Trimesh

	public static DHeightfield createHeightfield( DSpace space,
												  DHeightfieldData data, boolean bPlaceable ) {
		return DxHeightfield.dCreateHeightfield((DxSpace)space, (DxHeightfieldData)data, bPlaceable);
	}

	public static DHeightfieldData createHeightfieldData() {
		return DxHeightfieldData.dGeomHeightfieldDataCreate();
	}

	public static DHeightfield createTrimeshHeightfield( DSpace space,
														 DHeightfieldData data, boolean bPlaceable ) {
		return DxTrimeshHeightfield.dCreateHeightfield((DxSpace)space, (DxHeightfieldData)data, bPlaceable);
	}

	public static DTriMesh createTriMesh(DSpace space, DTriMeshData data) {
		return DxTriMesh.dCreateTriMesh((DxSpace)space, (DxTriMeshData)data,null, null, null);
	}

	public static DTriMesh createTriMesh(DSpace space, DTriMeshData data, DTriCallback callback,
										 DTriRayCallback rayCallback) {
		return DxTriMesh.dCreateTriMesh((DxSpace)space, (DxTriMeshData)data,
				callback, null, rayCallback);
	}

	@SuppressWarnings("deprecation")
	public static DTriMesh createTriMesh(DSpace space, DTriMeshData data, DTriCallback callback,
										 DTriMesh.DTriArrayCallback arrayCallback,
										 DTriRayCallback rayCallback) {
		return DxTriMesh.dCreateTriMesh((DxSpace)space, (DxTriMeshData)data,
				callback, arrayCallback, rayCallback);
	}

	public static DTriMeshData createTriMeshData() {
		return DxTriMeshData.dGeomTriMeshDataCreate();
	}

	//****************************************************************************

	@Deprecated // Make this "private" in 0.6.0
	protected OdeHelper() {}
}
