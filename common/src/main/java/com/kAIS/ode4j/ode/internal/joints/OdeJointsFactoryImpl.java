package com.kAIS.ode4j.ode.internal.joints;

import static com.kAIS.ode4j.ode.internal.Common.*;

import java.util.LinkedList;
import java.util.List;

import com.kAIS.ode4j.ode.DBody;
import com.kAIS.ode4j.ode.DContact;
import com.kAIS.ode4j.ode.DJoint;
import com.kAIS.ode4j.ode.DJointGroup;
import com.kAIS.ode4j.ode.DWorld;
import com.kAIS.ode4j.ode.internal.DxBody;
import com.kAIS.ode4j.ode.internal.DxWorld;

/**
 * Factory for Joints.
 */
public class OdeJointsFactoryImpl {

	//****************************************************************************
	// joints

	private <T extends DxJoint> T createJoint(T j, DJointGroup group)
	{
		//TODO move this into dxJoint constructor? (TZ)
		if (group != null) {
			((DxJointGroup)group).addJoint(j);
		}
		return j;
	}

	public DxJointBall dJointCreateBall (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointBall((DxWorld) w),group);
	}

	public DxJointConstrainedBall dJointCreateConstrainedBall (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointConstrainedBall(w),group);
		// 만약 컴파일러가 생성자 타입 불일치로 에러를 낸다면:
		// return createJoint( new DxJointConstrainedBall((DxWorld)w), group);
	}

	public DxJointHinge dJointCreateHinge (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointHinge((DxWorld) w),group);
	}

	public DxJointSlider dJointCreateSlider (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointSlider((DxWorld) w),group);
	}

	public DxJointContact dJointCreateContact (DWorld w, DJointGroup group,
											   final DContact c)
	{
		dAASSERT (w, c);
		DxJointContact j = createJoint(new DxJointContact((DxWorld) w), group);
		j.setContact(c);
		return j;
	}

	public DxJointHinge2 dJointCreateHinge2 (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint(new DxJointHinge2((DxWorld) w),group);
	}

	public DxJointUniversal dJointCreateUniversal (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint(new DxJointUniversal((DxWorld) w), group);
	}

	public DxJointPR dJointCreatePR (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointPR((DxWorld) w),group);
	}

	public DxJointPU  dJointCreatePU (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointPU((DxWorld) w),group);
	}

	public DxJointPiston  dJointCreatePiston (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointPiston((DxWorld) w),group);
	}

	public DxJointFixed dJointCreateFixed (DWorld id, DJointGroup group)
	{
		dAASSERT (id);
		return createJoint( new DxJointFixed((DxWorld)id),group);
	}

	public DxJointNull dJointCreateNull (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointNull((DxWorld) w),group);
	}

	public DxJointAMotor dJointCreateAMotor (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint(new DxJointAMotor((DxWorld) w),group);
	}

	public DxJointLMotor dJointCreateLMotor (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint(new DxJointLMotor ((DxWorld) w),group);
	}

	public DxJointPlane2D dJointCreatePlane2D (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointPlane2D((DxWorld) w),group);
	}

	public DxJointDBall dJointCreateDBall (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointDBall((DxWorld) w),group);
	}

	public DxJointDHinge dJointCreateDHinge (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointDHinge((DxWorld) w),group);
	}

	public DxJointTransmission dJointCreateTransmission (DWorld w, DJointGroup group)
	{
		dAASSERT (w);
		return createJoint( new DxJointTransmission((DxWorld) w),group);
	}

	// Joint 파괴: 팩토리 외부(OdeFactoryImpl 등)에서도 접근해야 해서 public static 으로
	public static void dJointDestroy (DxJoint j)
	{
		//dAASSERT (j);
		if ((j.flags & DxJoint.dJOINT_INGROUP)==0) {
			j.FinalizeAndDestroyJointInstance(true);
		}
	}

	public void dJointAttach (DxJoint joint, DxBody body1, DxBody body2)
	{
		joint.dJointAttach(body1, body2);
	}

	public static DJoint dConnectingJoint (DBody in_b1, DBody in_b2)
	{
		dAASSERT (in_b1!=null || in_b2!=null);

		DxBody b1, b2;

		if (in_b1 == null) {
			b1 = (DxBody) in_b2;
			b2 = (DxBody) in_b1;
		}
		else {
			b1 = (DxBody) in_b1;
			b2 = (DxBody) in_b2;
		}

		// look through b1's neighbour list for b2
		for (DxJointNode n=b1.firstjoint.get(); n!=null; n=n.next) {
			if (n.body == b2) return n.joint;
		}

		return null;
	}

	public static List<DJoint> dConnectingJointList (DxBody in_b1, DxBody in_b2)
	{
		dAASSERT (in_b1!=null || in_b2!=null);

		List<DJoint> out_list = new LinkedList<>();

		DxBody b1, b2;

		if (in_b1 == null) {
			b1 = in_b2;
			b2 = in_b1;
		}
		else {
			b1 = in_b1;
			b2 = in_b2;
		}

		for (DxJointNode n=b1.firstjoint.get(); n!=null; n=n.next) {
			if (n.body == b2)
				out_list.add(n.joint);
		}

		return out_list;
	}

	public boolean _dAreConnected (DBody b1, DBody b2)
	{
		for (DxJointNode n=((DxBody)b1).firstjoint.get(); n!=null; n=n.next) {
			if (n.body == b2) return true;
		}
		return false;
	}

	@SafeVarargs
	public final boolean _dAreConnectedExcluding(DBody b1, DBody b2, Class<? extends DJoint>... jointTypes)
	{
		for (DxJointNode n=((DxBody)b1).firstjoint.get(); n!=null; n=n.next) {
			if ( n.body == b2) {
				boolean found = false;
				Class<?> clsJoint = n.joint.getClass();
				for (Class<?> cls: jointTypes) {
					if ( cls == clsJoint )  {
						found = true;
						break;
					}
				}
				if (!found) return true;
			}
		}
		return false;
	}

	public OdeJointsFactoryImpl() {}
}
