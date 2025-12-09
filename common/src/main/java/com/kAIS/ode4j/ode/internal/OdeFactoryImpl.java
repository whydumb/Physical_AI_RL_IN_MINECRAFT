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
package com.kAIS.ode4j.ode.internal;

import com.kAIS.ode4j.ode.*;
import com.kAIS.ode4j.ode.internal.cpp4j.java.Ref;
import com.kAIS.ode4j.ode.internal.joints.OdeJointsFactoryImpl;
import com.kAIS.ode4j.ode.internal.joints.DxJoint;
import com.kAIS.ode4j.ode.internal.joints.DxJointNode;

import static com.kAIS.ode4j.ode.OdeMath.*;
import static com.kAIS.ode4j.ode.internal.ErrorHandler.*;
import static com.kAIS.ode4j.ode.internal.cpp4j.Cstdio.*;

/**
 * this source file is mostly concerned with the data structures, not the
 * numerics.
 */
public class OdeFactoryImpl {

	/** 내부 구현용 조인트 팩토리 (Dx* 타입 생성 전담) */
	private final OdeJointsFactoryImpl jointsFactory = new OdeJointsFactoryImpl();

	//****************************************************************************
	// debugging

	// see if an object list loops on itself (if so, it's bad).

	static <T extends DObject> boolean listHasLoops (Ref<T>  first)
	{
		if (first.get()==null || first.get().getNext()==null) return false;
		DObject a=first.get(),b=first.get().getNext();
		int skip=0;
		while (b != null) {
			if (a==b) return true;
			b = b.getNext();
			if (skip != 0) a = a.getNext();
			skip ^= 1;
		}
		return false;
	}

	// check the validity of the world data structures

	private static int g_world_check_tag_generator = 0;

	static int generateWorldCheckTag()
	{
		// Atomicity is not necessary here
		return ++g_world_check_tag_generator;
	}

	@SuppressWarnings("unused")
	static void checkWorld (DxWorld w)
	{
		DxBody b;
		DxJoint j;

		// check there are no loops
		if (listHasLoops (w.firstbody)) dDebug (0,"body list has loops");
		if (listHasLoops (w.firstjoint)) dDebug (0,"joint list has loops");

		// check lists are well-formed (check `tome' pointers)
		for (b=w.firstbody.get(); b!=null; b=(DxBody)b.getNext()) {
			if (b.getNext()!=null && b.getNext().getTome() != b.getNext())
				dDebug (0,"bad tome pointer in body list");
		}
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) {
			if (j.getNext()!=null && j.getNext().getTome() != j.getNext())
				dDebug (0,"bad tome pointer in joint list");
		}

		// check counts
		int nn = 0;
		for (b=w.firstbody.get(); b!=null; b=(DxBody)b.getNext()) nn++;
		if (w.nb != nn) dDebug (0,"body count incorrect");
		nn = 0;
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) nn++;
		if (w.nj != nn) dDebug (0,"joint count incorrect");

		// set all tag values to a known value
		int count = generateWorldCheckTag();
		for (b=w.firstbody.get(); b!=null; b=(DxBody)b.getNext()) b.tag = count;
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) j.tag = count;

		// check all body/joint world pointers are ok
		for (b=w.firstbody.get(); b!=null; b=(DxBody)b.getNext()) if (b.world != w)
			dDebug (0,"bad world pointer in body list");
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) if (j.world != w)
			dDebug (0,"bad world pointer in joint list");

		// check that every joint node appears in the joint lists of both bodies it
		// attaches
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) {
			for (int i=0; i<2; i++) {
				if (j.node[i].body!=null) {
					int ok = 0;
					for (DxJointNode n=j.node[i].body.firstjoint.get(); n!=null; n=n.next) {
						if (n.joint == j) ok = 1;
					}
					if (ok==0) dDebug (0,"joint not in joint list of attached body");
				}
			}
		}

		// check all body joint lists (correct body ptrs)
		for (b=w.firstbody.get(); b!=null; b=(DxBody)b.getNext()) {
			for (DxJointNode n=b.firstjoint.get(); n!=null; n=n.next) {
				if (n.joint.node[0] == n) {
					if (n.joint.node[1].body != b)
						dDebug (0,"bad body pointer in joint node of body list (1)");
				}
				else {
					if (n.joint.node[0].body != b)
						dDebug (0,"bad body pointer in joint node of body list (2)");
				}
				if (n.joint.tag != count) dDebug (0,"bad joint node pointer in body");
			}
		}

		// check all body pointers in joints, check they are distinct
		for (j=w.firstjoint.get(); j!=null; j=(DxJoint)j.getNext()) {
			if (j.node[0].body!=null && (j.node[0].body == j.node[1].body))
				dDebug (0,"non-distinct body pointers in joint");
			if ((j.node[0].body!=null && j.node[0].body.tag != count) ||
					(j.node[1].body!=null && j.node[1].body.tag != count))
				dDebug (0,"bad body pointer in joint");
		}
	}

	void dWorldCheck (DxWorld w)
	{
		checkWorld (w);
	}

	//****************************************************************************
	// joints – public API 쪽 팩토리 메서드 (인터페이스 타입 반환)

	public DAMotorJoint dJointCreateAMotor(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreateAMotor(world, group);
	}

	public DBallJoint dJointCreateBall(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreateBall(world, group);
	}

	public DContactJoint dJointCreateContact(DWorld world, DJointGroup group, DContact c) {
		return jointsFactory.dJointCreateContact(world, group, c);
	}

	public DDoubleBallJoint dJointCreateDBall(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreateDBall(world, group);
	}

	public DDoubleHingeJoint dJointCreateDHinge(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreateDHinge(world, group);
	}

	public DFixedJoint dJointCreateFixed(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreateFixed(world, group);
	}

	public DHingeJoint dJointCreateHinge(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreateHinge(world, group);
	}

	public DHinge2Joint dJointCreateHinge2(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreateHinge2(world, group);
	}

	public DLMotorJoint dJointCreateLMotor(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreateLMotor(world, group);
	}

	public DNullJoint dJointCreateNull(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreateNull(world, group);
	}

	public DPistonJoint dJointCreatePiston(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreatePiston(world, group);
	}

	public DPlane2DJoint dJointCreatePlane2D(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreatePlane2D(world, group);
	}

	public DPRJoint dJointCreatePR(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreatePR(world, group);
	}

	public DPUJoint dJointCreatePU(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreatePU(world, group);
	}

	public DSliderJoint dJointCreateSlider(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreateSlider(world, group);
	}

	public DTransmissionJoint dJointCreateTransmission(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreateTransmission(world, group);
	}

	public DUniversalJoint dJointCreateUniversal(DWorld world, DJointGroup group) {
		return jointsFactory.dJointCreateUniversal(world, group);
	}

	//****************************************************************************
	// connection helpers – OdeHelper 에서 사용

	public boolean _dAreConnected(DBody b1, DBody b2) {
		return jointsFactory._dAreConnected(b1, b2);
	}

	@SafeVarargs
	public final boolean _dAreConnectedExcluding(DBody b1, DBody b2,
												 Class<? extends DJoint>... jointTypes) {
		return jointsFactory._dAreConnectedExcluding(b1, b2, jointTypes);
	}


	//****************************************************************************
	// testing

	private static final int NUM = 100;

	private static boolean DO = false;
	private static void DO_printf(String msg, Object... args) {
		if (DO) {
			printf(msg, args);
		}
	}

	//extern "C"
	public void dTestDataStructures()
	{
		int i;
		DO_printf ("testDynamicsStuff()\n");

		DxBody[] body = new DxBody[NUM];
		int nb = 0;
		DxJoint[] joint = new DxJoint[NUM];
		int nj = 0;

		for (i=0; i<NUM; i++) body[i] = null;
		for (i=0; i<NUM; i++) joint[i] = null;

		DO_printf ("creating world\n");
		DxWorld w = DxWorld.dWorldCreate();
		checkWorld (w);

		for (int round = 0; round < 1000; ++round) {
			if (nb < NUM && dRandReal() > 0.5) {
				DO_printf ("creating body\n");
				body[nb] = DxBody.dBodyCreate (w);
				DO_printf ("\t--> %s\n",body[nb].toString());
				nb++;
				checkWorld (w);
				DO_printf ("%d BODIES, %d JOINTS\n",nb,nj);
			}
			if (nj < NUM && nb > 2 && dRandReal() > 0.5) {
				DxBody b1 = body [(int) (dRand() % nb)];
				DxBody b2 = body [(int) (dRand() % nb)];
				if (b1 != b2) {
					DO_printf ("creating joint, attaching to %s,%s\n",b1.toString(),b2.toString());
					joint[nj] = jointsFactory.dJointCreateBall(w,null); // 내부용 팩토리 직접 사용
					DO_printf ("\t-->%s\n",joint[nj].toString());
					checkWorld (w);
					joint[nj].dJointAttach (b1,b2);
					nj++;
					checkWorld (w);
					DO_printf ("%d BODIES, %d JOINTS\n",nb,nj);
				}
			}
			if (nj > 0 && nb > 2 && dRandReal() > 0.5) {
				DxBody b1 = body [(int) (dRand() % nb)];
				DxBody b2 = body [(int) (dRand() % nb)];
				if (b1 != b2) {
					int k = (int) (dRand() % nj);
					DO_printf ("reattaching joint %s\n",joint[k].toString());
					joint[k].dJointAttach (b1,b2);
					checkWorld (w);
					DO_printf ("%d BODIES, %d JOINTS\n",nb,nj);
				}
			}
			if (nb > 0 && dRandReal() > 0.5) {
				int k = (int) (dRand() % nb);
				DO_printf ("destroying body %s\n",body[k].toString());
				body[k].dBodyDestroy ();
				checkWorld (w);
				for (; k < (NUM-1); k++) body[k] = body[k+1];
				nb--;
				DO_printf ("%d BODIES, %d JOINTS\n",nb,nj);
			}
			if (nj > 0 && dRandReal() > 0.5) {
				int k = (int) (dRand() % nj);
				DO_printf ("destroying joint %s\n",joint[k].toString());
				OdeJointsFactoryImpl.dJointDestroy (joint[k]);
				checkWorld (w);
				for (; k < (NUM-1); k++) joint[k] = joint[k+1];
				nj--;
				DO_printf ("%d BODIES, %d JOINTS\n",nb,nj);
			}
		}
	}

	//****************************************************************************
	// configuration

	private static void REGISTER_EXTENSION(String s) {
		ode_configuration += s + " ";
	}

	private static String ode_configuration = "ODE ";

	static {
		if (Common.dNODEBUG)
			REGISTER_EXTENSION( "ODE_EXT_no_debug" );

		if (Common.dTRIMESH_ENABLED) {
			REGISTER_EXTENSION( "ODE_EXT_trimesh" );

			if (Common.dTRIMESH_OPCODE) {
				REGISTER_EXTENSION( "ODE_EXT_opcode" );

				if (Common.dTRIMESH_16BIT_INDICES)
					REGISTER_EXTENSION( "ODE_OPC_16bit_indices" );

				if (!Common.dTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER)
					REGISTER_EXTENSION( "ODE_OPC_new_collider" );
			}

			if (Common.dTRIMESH_GIMPACT) {
				REGISTER_EXTENSION( "ODE_EXT_gimpact" );
			}
		}

		REGISTER_EXTENSION("ODE_EXT_libccd");
		REGISTER_EXTENSION("ODE_EXT_inelastic_collisions");

		ode_configuration += "ODE_double_precision";
	}

	public String _dGetConfiguration ()
	{
		return ode_configuration;
	}

	public boolean _dCheckConfiguration( final String extension )
	{
		int start;
		int where;
		int terminator;

		if (extension.indexOf(' ') >= 0 || extension.length() == 0)
			return true;  // TODO: 아마 false 가 더 맞지만, 원본 코드 유지

		final String config = com.kAIS.ode4j.ode.OdeHelper.getConfiguration();
		final int ext_length = extension.length();

		start = 0;
		for (  ; ;  )
		{
			where = config.indexOf(extension, start);
			if (where == -1)
				break;

			terminator = where + ext_length;

			if ( (where == start || config.charAt(where - 1) == ' ') &&
					(terminator == config.length() || config.charAt(terminator) == ' ') )
			{
				return true;
			}

			start = terminator;
		}

		return false;
	}

	public OdeFactoryImpl() {}
}
