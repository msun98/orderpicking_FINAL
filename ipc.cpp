#include "ipc.h"

IPC::IPC(QObject *parent)
    : QObject(parent)
    , shm_status("slamnav_status")
    , shm_path("slamnav_path")
    , shm_map("slamnav_map")
    , shm_obs("slamnav_obs")
    , shm_cam0("slamnav_cam0")
    , shm_cam1("slamnav_cam1")
    , shm_cam_color0("slamnav_cam_color0")
    , shm_cam_color1("slamnav_cam_color1")
    , shm_loc_status("slamnav_loc_status")
    , shm_call_status("slamnav_call_status")
    , shm_call_loc("slamnav_call_loc")
    , shm_check_travel("slamnav_check_travel")
{
    // msg tick clear, check for new data
    tick = 0;

    if (!shm_status.create(sizeof(IPC::STATUS), QSharedMemory::ReadWrite) && shm_status.error() == QSharedMemory::AlreadyExists)
    {
        printf("attach shared memory, key: slamnav_status, size: %ld\n", sizeof(IPC::STATUS));
        shm_status.attach();
    }
    else
    {
        printf("create shared memory, key: slamnav_status, size: %ld\n", sizeof(IPC::STATUS));
    }

    if (!shm_path.create(sizeof(IPC::PATH), QSharedMemory::ReadWrite) && shm_path.error() == QSharedMemory::AlreadyExists)
    {
        printf("attach shared memory, key: slamnav_path, size: %ld\n", sizeof(IPC::PATH));
        shm_path.attach();
    }
    else
    {
        printf("create shared memory, key: slamnav_path, size: %ld\n", sizeof(IPC::PATH));
    }

    if (!shm_map.create(sizeof(IPC::MAP), QSharedMemory::ReadWrite) && shm_map.error() == QSharedMemory::AlreadyExists)
    {
        printf("attach shared memory, key: slamnav_map, size: %ld\n", sizeof(IPC::MAP));
        shm_map.attach();
    }
    else
    {
        printf("create shared memory, key: slamnav_map, size: %ld\n", sizeof(IPC::MAP));
    }

    if (!shm_obs.create(sizeof(IPC::MAP), QSharedMemory::ReadWrite) && shm_obs.error() == QSharedMemory::AlreadyExists)
    {
        printf("attach shared memory, key: slamnav_obs, size: %ld\n", sizeof(IPC::MAP));
        shm_obs.attach();
    }
    else
    {
        printf("create shared memory, key: slamnav_obs, size: %ld\n", sizeof(IPC::MAP));
    }

    if (!shm_cam0.create(sizeof(IPC::IMG), QSharedMemory::ReadWrite) && shm_cam0.error() == QSharedMemory::AlreadyExists)
    {
        printf("attach shared memory, key: slamnav_cam0, size: %ld\n", sizeof(IPC::IMG));
        shm_cam0.attach();
    }
    else
    {
        printf("create shared memory, key: slamnav_cam0, size: %ld\n", sizeof(IPC::IMG));
    }

    if (!shm_cam1.create(sizeof(IPC::IMG), QSharedMemory::ReadWrite) && shm_cam1.error() == QSharedMemory::AlreadyExists)
    {
        printf("attach shared memory, key: slamnav_cam1, size: %ld\n", sizeof(IPC::IMG));
        shm_cam1.attach();
    }
    else
    {
        printf("create shared memory, key: slamnav_cam1, size: %ld\n", sizeof(IPC::IMG));
    }

    if (!shm_cam_color0.create(sizeof(IPC::IMG_COLOR), QSharedMemory::ReadWrite) && shm_cam_color0.error() == QSharedMemory::AlreadyExists)
    {
        printf("attach shared memory, key: slamnav_cam_color0, size: %ld\n", sizeof(IPC::IMG_COLOR));
        shm_cam_color0.attach();
    }
    else
    {
        printf("create shared memory, key: slamnav_cam_color0, size: %ld\n", sizeof(IPC::IMG_COLOR));
    }

    if (!shm_cam_color1.create(sizeof(IPC::IMG_COLOR), QSharedMemory::ReadWrite) && shm_cam_color1.error() == QSharedMemory::AlreadyExists)
    {
        printf("attach shared memory, key: slamnav_cam_color1, size: %ld\n", sizeof(IPC::IMG_COLOR));
        shm_cam_color1.attach();
    }
    else
    {
        printf("create shared memory, key: slamnav_cam_color1, size: %ld\n", sizeof(IPC::IMG_COLOR));
    }

    if (!shm_loc_status.create(sizeof(IPC::LOC_STATUS), QSharedMemory::ReadWrite) && shm_loc_status.error() == QSharedMemory::AlreadyExists)
    {
        printf("attach shared memory, key: shm_loc_status, size: %ld\n", sizeof(IPC::LOC_STATUS));
        shm_loc_status.attach();
    }
    else
    {
        printf("create shared memory, key: shm_loc_status, size: %ld\n", sizeof(IPC::LOC_STATUS));
    }

    if (!shm_call_status.create(sizeof(IPC::CALL_STATUS), QSharedMemory::ReadWrite) && shm_call_status.error() == QSharedMemory::AlreadyExists)
    {
        printf("attach shared memory, key: shm_call_status, size: %ld\n", sizeof(IPC::CALL_STATUS));
        shm_call_status.attach();
    }
    else
    {
        printf("create shared memory, key: shm_call_status, size: %ld\n", sizeof(IPC::CALL_STATUS));
    }

    if (!shm_call_loc.create(sizeof(IPC::CALL_LOC), QSharedMemory::ReadWrite) && shm_call_loc.error() == QSharedMemory::AlreadyExists)
    {
        printf("attach shared memory, key: shm_call_loc, size: %ld\n", sizeof(IPC::CALL_LOC));
        shm_call_loc.attach();
    }
    else
    {
        printf("create shared memory, key: shm_call_loc, size: %ld\n", sizeof(IPC::CALL_LOC));
    }

    if (!shm_check_travel.create(sizeof(IPC::CHECK_TRAVEL), QSharedMemory::ReadWrite) && shm_check_travel.error() == QSharedMemory::AlreadyExists)
    {
        printf("attach shared memory, key: shm_check_travel, size: %ld\n", sizeof(IPC::CHECK_TRAVEL));
        shm_check_travel.attach();
    }
    else
    {
        printf("create shared memory, key: shm_check_travel, size: %ld\n", sizeof(IPC::CHECK_TRAVEL));
    }
}

IPC::~IPC()
{
    if(shm_status.detach())
    {
        printf("detach shared memory, key: slamnav_status\n");
    }

    if(shm_path.detach())
    {
        printf("detach shared memory, key: slamnav_path\n");
    }

    if(shm_map.detach())
    {
        printf("detach shared memory, key: slamnav_map\n");
    }

    if(shm_obs.detach())
    {
        printf("detach shared memory, key: slamnav_obs\n");
    }

    if(shm_cam0.detach())
    {
        printf("detach shared memory, key: slamnav_cam0\n");
    }

    if(shm_cam1.detach())
    {
        printf("detach shared memory, key: slamnav_cam1\n");
    }

    if(shm_loc_status.detach())
    {
        printf("detach shared memory, key: shm_loc_status\n");
    }

    if(shm_call_status.detach())
    {
        printf("detach shared memory, key: shm_call_status\n");
    }

    if(shm_call_loc.detach())
    {
        printf("detach shared memory, key: shm_call_loc\n");
    }

    if(shm_check_travel.detach())
    {
        printf("detach shared memory, key: shm_check_travel\n");
    }
}

IPC::STATUS IPC::get_status()
{
    IPC::STATUS res;

    shm_status.lock();
    memcpy(&res, (char*)shm_status.constData(), sizeof(IPC::STATUS));
    shm_status.unlock();

    return res;
}

IPC::PATH IPC::get_path()
{
    IPC::PATH res;

    shm_path.lock();
    memcpy(&res, (char*)shm_path.constData(), sizeof(IPC::PATH));
    shm_path.unlock();

    return res;
}

IPC::MAP IPC::get_map()
{
    IPC::MAP res;

    shm_map.lock();
    memcpy(&res, (char*)shm_map.constData(), sizeof(IPC::MAP));
    shm_map.unlock();

    return res;
}

IPC::MAP IPC::get_obs()
{
    IPC::MAP res;

    shm_obs.lock();
    memcpy(&res, (char*)shm_obs.constData(), sizeof(IPC::MAP));
    shm_obs.unlock();

    return res;
}

IPC::IMG IPC::get_cam0()
{
    IPC::IMG res;

    shm_cam0.lock();
    memcpy(&res, (char*)shm_cam0.constData(), sizeof(IPC::IMG));
    shm_cam0.unlock();

    return res;
}

IPC::IMG IPC::get_cam1()
{
    IPC::IMG res;

    shm_cam1.lock();
    memcpy(&res, (char*)shm_cam1.constData(), sizeof(IPC::IMG));
    shm_cam1.unlock();

    return res;
}

IPC::LOC_STATUS IPC::get_loc_status()
{
    IPC::LOC_STATUS res;

    shm_loc_status.lock();
    memcpy(&res, (char*)shm_loc_status.constData(), sizeof(IPC::LOC_STATUS));
    shm_loc_status.unlock();

    return res;
}

IPC::CALL_STATUS IPC::get_call_status()
{
    IPC::CALL_STATUS res;

    shm_call_status.lock();
    memcpy(&res, (char*)shm_call_status.constData(), sizeof(IPC::CALL_STATUS));
    shm_call_status.unlock();

    return res;
}

IPC::CALL_LOC IPC::get_call_loc()
{
    IPC::CALL_LOC res;

    shm_call_loc.lock();
    memcpy(&res, (char*)shm_call_loc.constData(), sizeof(IPC::CALL_LOC));
    shm_call_loc.unlock();

    return res;
}

IPC::CHECK_TRAVEL IPC::get_check_travel()
{
    IPC::CHECK_TRAVEL res;

    shm_check_travel.lock();
    memcpy(&res, (char*)shm_check_travel.constData(), sizeof(IPC::CHECK_TRAVEL));
    shm_check_travel.unlock();

    return res;
}

void IPC::set_status(IPC::STATUS val)
{
    shm_status.lock();
    val.tick = ++tick;
    memcpy((char*)shm_status.data(), &val, sizeof(IPC::STATUS));
    shm_status.unlock();
}

void IPC::set_path(IPC::PATH val)
{
    shm_path.lock();
    val.tick = ++tick;
    memcpy((char*)shm_path.data(), &val, sizeof(IPC::PATH));
    shm_path.unlock();
}

void IPC::set_map(IPC::MAP val)
{
    shm_map.lock();
    val.tick = ++tick;
    memcpy((char*)shm_map.data(), &val, sizeof(IPC::MAP));
    shm_map.unlock();
}

void IPC::set_obs(IPC::MAP val)
{
    shm_obs.lock();
    val.tick = ++tick;
    memcpy((char*)shm_obs.data(), &val, sizeof(IPC::MAP));
    shm_obs.unlock();
}

void IPC::set_cam0(IPC::IMG val)
{
    shm_cam0.lock();
    val.tick = ++tick;
    memcpy((char*)shm_cam0.data(), &val, sizeof(IPC::IMG));
    shm_cam0.unlock();
}

void IPC::set_cam1(IPC::IMG val)
{
    shm_cam1.lock();
    val.tick = ++tick;
    memcpy((char*)shm_cam1.data(), &val, sizeof(IPC::IMG));
    shm_cam1.unlock();
}

void IPC::set_cam_color0(IPC::IMG_COLOR val)
{
    shm_cam_color0.lock();
    val.tick = ++tick;
    memcpy((char*)shm_cam_color0.data(), &val, sizeof(IPC::IMG_COLOR));
    shm_cam_color0.unlock();
}

void IPC::set_cam_color1(IPC::IMG_COLOR val)
{
    shm_cam_color1.lock();
    val.tick = ++tick;
    memcpy((char*)shm_cam_color1.data(), &val, sizeof(IPC::IMG_COLOR));
    shm_cam_color1.unlock();
}

void IPC::set_loc_status(IPC::LOC_STATUS val)
{
    shm_loc_status.lock();
    val.tick = ++tick;
    memcpy((char*)shm_loc_status.data(), &val, sizeof(IPC::LOC_STATUS));
    shm_loc_status.unlock();
}

void IPC::set_call_status(IPC::CALL_STATUS val)
{
    shm_call_status.lock();
    val.tick = ++tick;
    memcpy((char*)shm_call_status.data(), &val, sizeof(IPC::CALL_STATUS));
    shm_call_status.unlock();
}

void IPC::set_call_loc(IPC::CALL_LOC val)
{
    shm_call_loc.lock();
    val.tick = ++tick;
    memcpy((char*)shm_call_loc.data(), &val, sizeof(IPC::CALL_LOC));
    shm_call_loc.unlock();
}

void IPC::set_check_travel(IPC::CHECK_TRAVEL val)
{
    shm_check_travel.lock();
    val.tick = ++tick;
    memcpy((char*)shm_check_travel.data(), &val, sizeof(IPC::CHECK_TRAVEL));
    shm_check_travel.unlock();
}
