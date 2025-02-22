using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using System;
using System.Runtime.InteropServices;
using System.Linq;
using UnityEngine.VFX;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using TMPro;


public class Stereovision : MonoBehaviour
{
    [DllImport ("DisparityFiltering", EntryPoint = "processDisparity")]
    private static extern int processDisparity(float[] pointCloudData, int height, int width, int[] leftImage, int[] rightImage);
    [DllImport ("DisparityFiltering", EntryPoint = "getDirectionVectors")]
    private static extern int getDirectionVectors(float[] dirVecs, int height, int width, int[] image);

    public int everyOther = 1;
    public GameObject cloudRegion;

    private GameObject obj;
    private VisualEffect cloudVFX;

    private Image leftImage;
    private Image rightImage;
    private Image thermalImage;
    private DroneInstance droneInstance;
    private Point[] pointCloud;

    private GraphicsBuffer pointsBuffer;
    VisualEffect vfx;
    public float pointSize = 0.1f;
    public float boundsSize = 10f;
    public float cellSize = 0.5f;
    public Color defaultColor = Color.white;
    public TextMeshProUGUI pointText;

    [VFXType(VFXTypeAttribute.Usage.GraphicsBuffer), StructLayout(LayoutKind.Sequential)]
    public struct Point {
        public Vector3 position;
        public Color color;
    }

    struct HashAndIndex : IComparable<HashAndIndex> {
        public int Hash;
        public int Index;

        public int CompareTo(HashAndIndex other) {
            return Hash.CompareTo(other.Hash);
        }
    }

    #region Native Objects
    NativeArray<Point> pointsNative;
    NativeArray<HashAndIndex> hashAndIndices;
    NativeArray<int> cellCountNative;
    NativeHashSet<int> activeHashes;
    NativeArray<Point> pointsNativeChecking;
    NativeArray<HashAndIndex> hashAndIndicesChecking;
    NativeArray<int> cellCountNativeChecking;
    NativeHashSet<int> activeHashesChecking;
    NativeList<int> resultIndices;
    NativeList<Point> joinedCloudNative;
    #endregion

    // Start is called before the first frame update
    void Start()
    {
        obj = GameObject.Instantiate(cloudRegion, Vector3.zero, Quaternion.identity);
        cloudVFX = obj.GetComponent<VisualEffect>();
        droneInstance = transform.parent.GetComponent<DroneInstance>();
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyUp("space") && leftImage.data != null && rightImage.data != null) {
            // Process disparity and reconstruct 3D mesh when spacebar is pressed
            int height = leftImage.height;
            int width = leftImage.width;

            float[] pointCloudData = new float[height * width * 3];
            float startTime = Time.realtimeSinceStartup;
            Debug.Log(processDisparity(pointCloudData, height, width, leftImage.data, rightImage.data)); // Returns point cloud data
            Debug.Log("Time to generate point cloud: " + (Time.realtimeSinceStartup - startTime).ToString("f6"));

            // Organize floats into Vector3
            Point[] pointCloud = new Point[height * width / (everyOther + 1)];
            int index = 0;
            int count = 0;
            for (int i = 0; i < pointCloud.Length; i++) {
                if (everyOther != 0) {
                    if (index != everyOther) {
                        index++;
                        continue;
                    } else {
                        index = 0;
                    }
                }
                Vector3 point = new Vector3(pointCloudData[i * 3], pointCloudData[i * 3 + 1], pointCloudData[i * 3 + 2]) / 1000;
                if (float.IsNaN(point.x) || float.IsNaN(point.y) || float.IsNaN(point.z) || float.IsInfinity(point.x) || float.IsInfinity(point.y) || float.IsInfinity(point.z)) {
                    pointCloud[count++] = new Point() { position = Vector3.zero, color = defaultColor };
                } else {
                    pointCloud[count++] = new Point() { position = transform.TransformPoint(point), color = new Color(UnityEngine.Random.value , UnityEngine.Random.value, UnityEngine.Random.value, 1) };
                }
            }

            Reconstruct3D(pointCloud);
        }
    }

    void Reconstruct3D(Point[] newCloud) {
        Debug.Log("reconstructing");

        float startTime = Time.realtimeSinceStartup;
        pointCloud = JoinClouds(newCloud);
        Debug.Log("Time to join clouds: " + (Time.realtimeSinceStartup - startTime).ToString("f6"));

        pointsBuffer = new GraphicsBuffer(GraphicsBuffer.Target.Structured, pointCloud.Length, Marshal.SizeOf(typeof(Point)));
        pointsBuffer.SetData(pointCloud);

        cloudVFX.SetGraphicsBuffer(Shader.PropertyToID("PointsBuffer"), pointsBuffer);
        cloudVFX.SetUInt(Shader.PropertyToID("ParticleCount"), (uint)pointCloud.Length);
        cloudVFX.SetFloat(Shader.PropertyToID("ParticleSize"), pointSize);
        cloudVFX.SetFloat(Shader.PropertyToID("BoundsSize"), boundsSize);

        cloudVFX.Reinit();
        pointText.text = "Points: " + pointCloud.Length.ToString("###,###,###.");
    }

    private void OnDestroy() {
        if (pointsBuffer != null) pointsBuffer.Release();
        if (pointsNative.IsCreated) pointsNative.Dispose();
        if (pointsNativeChecking.IsCreated) pointsNativeChecking.Dispose();
		if (hashAndIndices.IsCreated) hashAndIndices.Dispose();
		if (hashAndIndicesChecking.IsCreated) hashAndIndicesChecking.Dispose();
		if (resultIndices.IsCreated) resultIndices.Dispose();
		if (joinedCloudNative.IsCreated) joinedCloudNative.Dispose();
		if (cellCountNative.IsCreated) cellCountNative.Dispose();
		if (cellCountNativeChecking.IsCreated) cellCountNativeChecking.Dispose();
		if (activeHashes.IsCreated) activeHashes.Dispose();
		if (activeHashesChecking.IsCreated) activeHashesChecking.Dispose();
    }

    private Point[] JoinClouds(Point[] newCloud) {
        if (pointCloud == null) {
            //hash = new Hash(1, 1000000);
            //checkingHash = new Hash(1, 1000000);
            //hash.Create(newCloud);

            CreateMainHash(newCloud);

            return newCloud;
        } else {
            // Compare new cloud and existing cloud points. Whichever has higher concentration in grid point will remain
            //hash.Create(pointCloud);
            CreateCheckingHash(newCloud);
            
		    if (joinedCloudNative.IsCreated) joinedCloudNative.Dispose();
            joinedCloudNative = new NativeList<Point>(Allocator.Persistent);
            NativeHashSet<int> addedHashes = new NativeHashSet<int>(pointCloud.Length + newCloud.Length, Allocator.TempJob);

            foreach (int hash in activeHashesChecking) {
                if (activeHashes.Contains(hash)) { // Existing cloud already has this grid cell
                    if (cellCountNative[hash] > cellCountNativeChecking[hash]) { // Find which has the higher density
                        // Original is higher density

                        var queryHashJob = new QueryHashJob {
                            hashAndIndices = hashAndIndices,
                            queryHash = hash,
                            resultIndices = new NativeList<int>(Allocator.TempJob)
                        };

                        JobHandle queryHashJobHandle = queryHashJob.Schedule();

                        queryHashJobHandle.Complete();

                        if (resultIndices.IsCreated) resultIndices.Dispose();
                        resultIndices = queryHashJob.resultIndices;

                        var findPointsJob = new FindPointsJob {
                            resultIndices = resultIndices.AsArray(),
                            cloud = pointsNative,
                            joinedCloud = joinedCloudNative
                        };
                        // Adds points at resultIndices to joinedCloudNative

                        JobHandle findPointsJobHandle = findPointsJob.Schedule();

                        findPointsJobHandle.Complete();

                        joinedCloudNative = findPointsJob.joinedCloud;
                    } else {
                        // New is higher density or equal

                        var queryHashJob = new QueryHashJob {
                            hashAndIndices = hashAndIndicesChecking,
                            queryHash = hash,
                            resultIndices = new NativeList<int>(Allocator.TempJob)
                        };

                        JobHandle queryHashJobHandle = queryHashJob.Schedule();

                        queryHashJobHandle.Complete();

                        if (resultIndices.IsCreated) resultIndices.Dispose();
                        resultIndices = queryHashJob.resultIndices;

                        var findPointsJob = new FindPointsJob {
                            resultIndices = resultIndices.AsArray(),
                            cloud = pointsNativeChecking,
                            joinedCloud = joinedCloudNative
                        };

                        JobHandle findPointsJobHandle = findPointsJob.Schedule();

                        findPointsJobHandle.Complete();

                        joinedCloudNative = findPointsJob.joinedCloud;
                    }
                } else {
                    // Not in already existing cell

                    var queryHashJob = new QueryHashJob {
                        hashAndIndices = hashAndIndicesChecking,
                        queryHash = hash,
                        resultIndices = new NativeList<int>(Allocator.TempJob)
                    };

                    JobHandle queryHashJobHandle = queryHashJob.Schedule();

                    queryHashJobHandle.Complete();

                    if (resultIndices.IsCreated) resultIndices.Dispose();
                    resultIndices = queryHashJob.resultIndices;

                    var findPointsJob = new FindPointsJob {
                        resultIndices = resultIndices.AsArray(),
                        cloud = pointsNativeChecking,
                        joinedCloud = joinedCloudNative
                    };

                    JobHandle findPointsJobHandle = findPointsJob.Schedule();

                    findPointsJobHandle.Complete();

                    joinedCloudNative = findPointsJob.joinedCloud;
                }

                addedHashes.Add(hash);
            }

            foreach (int hash in activeHashes) {
                if (!addedHashes.Contains(hash)) {
                    var queryHashJob = new QueryHashJob {
                        hashAndIndices = hashAndIndices,
                        queryHash = hash,
                        resultIndices = new NativeList<int>(Allocator.TempJob)
                    };

                    JobHandle queryHashJobHandle = queryHashJob.Schedule();

                    queryHashJobHandle.Complete();

                    if (resultIndices.IsCreated) resultIndices.Dispose();
                    resultIndices = queryHashJob.resultIndices;

                    var findPointsJob = new FindPointsJob {
                        resultIndices = resultIndices.AsArray(),
                        cloud = pointsNative,
                        joinedCloud = joinedCloudNative
                    };
                    // Adds points at resultIndices to joinedCloudNative

                    JobHandle findPointsJobHandle = findPointsJob.Schedule();

                    findPointsJobHandle.Complete();

                    joinedCloudNative = findPointsJob.joinedCloud;
                }
            }

            Point[] joinedCloud = joinedCloudNative.AsArray().ToArray();
		    if (joinedCloudNative.IsCreated) joinedCloudNative.Dispose();
		    if (addedHashes.IsCreated) addedHashes.Dispose();
		    if (resultIndices.IsCreated) resultIndices.Dispose();
            CreateMainHash(joinedCloud);

            return joinedCloud;
        }
    }

    void CreateMainHash(Point[] cloud) {
        pointsNative = new NativeArray<Point>(cloud.Length, Allocator.Persistent);
        hashAndIndices = new NativeArray<HashAndIndex>(cloud.Length, Allocator.Persistent);
        cellCountNative = new NativeArray<int>(cloud.Length, Allocator.Persistent);
        activeHashes = new NativeHashSet<int>(cloud.Length, Allocator.Persistent);

        for (int i = 0; i < cloud.Length; i++) {
            pointsNative[i] = cloud[i];
        }

        var hashJob = new HashPointsJob {
            points = pointsNative,
            cellSize = cellSize,
            hashAndIndices = hashAndIndices,
            tableSize = cloud.Length
        };

        JobHandle hashJobHandle = hashJob.Schedule(pointsNative.Length, 64);

        var sortJob = new SortHashCodesJob {
            hashAndIndices = hashAndIndices,
            activeHashes = activeHashes,
            cellCount = cellCountNative
        };

        JobHandle sortJobHandle = sortJob.Schedule(hashJobHandle);

        sortJobHandle.Complete();
    }

    void CreateCheckingHash(Point[] cloud) {
        pointsNativeChecking = new NativeArray<Point>(cloud.Length, Allocator.Persistent);
        hashAndIndicesChecking = new NativeArray<HashAndIndex>(cloud.Length, Allocator.Persistent);
        cellCountNativeChecking = new NativeArray<int>(pointCloud.Length, Allocator.Persistent);
        activeHashesChecking = new NativeHashSet<int>(pointCloud.Length, Allocator.Persistent);

        for (int i = 0; i < cloud.Length; i++) {
            pointsNativeChecking[i] = cloud[i];
        }

        var hashJob = new HashPointsJob {
            points = pointsNativeChecking,
            cellSize = cellSize,
            hashAndIndices = hashAndIndicesChecking,
            tableSize = pointCloud.Length
        };

        JobHandle hashJobHandle = hashJob.Schedule(pointsNativeChecking.Length, 64);

        var sortJob = new SortHashCodesJob {
            hashAndIndices = hashAndIndicesChecking,
            activeHashes = activeHashesChecking,
            cellCount = cellCountNativeChecking
        };

        JobHandle sortJobHandle = sortJob.Schedule(hashJobHandle);

        sortJobHandle.Complete();
    }

    private void ProjectThermal() {
        int height = thermalImage.height;
        int width = thermalImage.width;

        float[] dirVecData = new float[height * width * 3];
        Debug.Log(getDirectionVectors(dirVecData, height, width, thermalImage.data)); // Returns the thermal image's direction vectors ungrouped

        // Organize floats into Vector3's
        Vector3[] dirVecs = new Vector3[height * width];
        for (int i = 0; i < height; i++) {
            dirVecs[i] = transform.TransformDirection(new Vector3(dirVecData[i * 3], dirVecData[i * 3 + 1], dirVecData[i * 3 + 2]));
        }
    }

    public void SetLeftImage(int height, int width, string encoding, byte[] data) {
        leftImage = new Image(height, width, encoding, data);
    }

    public void SetRightImage(int height, int width, string encoding, byte[] data) {
        rightImage = new Image(height, width, encoding, data);
    }

    public void SetThermalImage(int height, int width, string encoding, byte[] data) {
        thermalImage = new Image(height, width, encoding, data);
    }

    public static int Hash(int3 gridPos, int tableSize) {
        unchecked {
            return math.abs((gridPos.x * 92837111) ^ (gridPos.y * 689287499) ^ (gridPos.z * 283923481)) % tableSize;
		}
    }

    [BurstCompile]
    struct HashPointsJob : IJobParallelFor {
        [ReadOnly] public NativeArray<Point> points;
        public NativeArray<HashAndIndex> hashAndIndices;
        public float cellSize;
        public int tableSize;

        public void Execute(int index) {
            Point point = points[index];
            int hash = Hash(GridPosition(point.position, cellSize), tableSize);

            hashAndIndices[index] = new HashAndIndex { Hash = hash, Index = index };
        }
    }

	static int3 GridPosition(float3 position, float cellSize) {
		return new int3(math.floor(position / cellSize));
	}

	[BurstCompile]
    struct SortHashCodesJob : IJob {
        public NativeArray<HashAndIndex> hashAndIndices;
        public NativeArray<int> cellCount;
        public NativeHashSet<int> activeHashes;

        public void Execute() {
            hashAndIndices.Sort();

            for (int i = 0; i < hashAndIndices.Length; i++) {
                cellCount[hashAndIndices[i].Hash]++;

                if (!activeHashes.Contains(hashAndIndices[i].Hash)) // Efficient check
                {
                    activeHashes.Add(hashAndIndices[i].Hash);
                }
            }
        }
    }

	[BurstCompile]
    struct QueryJob : IJob {
        [ReadOnly] public NativeArray<Point> points;
        [ReadOnly] public NativeArray<HashAndIndex> hashAndIndices;
        public float3 queryPosition;
        public float queryRadius;
        public float cellSize;
        public NativeList<int> resultIndices;
        public int tableSize;

        public void Execute() {
            float radiusSquared = queryRadius * queryRadius;
            int3 minGridPos = GridPosition(queryPosition - queryRadius, cellSize);
			int3 maxGridPos = GridPosition(queryPosition + queryRadius, cellSize);

            for (int x = minGridPos.x; x <= maxGridPos.x; x++) {
				for (int y = minGridPos.y; y <= maxGridPos.y; y++) {
					for (int z = minGridPos.z; z <= maxGridPos.z; z++) {
                        int3 gridPos = new(x, y, z);
                        int hash = Hash(gridPos, tableSize);

                        int startIndex = BinarySearchFirst(hashAndIndices, hash);

                        if (startIndex < 0) continue; // No points in this grid cell

                        // Loop through all the points in this grid cell that have the same hash
                        for (int i = startIndex; i < hashAndIndices.Length && hashAndIndices[i].Hash == hash; i++) {
                            int pointIndex = hashAndIndices[i].Index;
                            Point point = points[pointIndex];
                            float3 toPoint = new float3(point.position.x, point.position.y, point.position.z) - queryPosition;

                            if (math.lengthsq(toPoint) <= radiusSquared) {
                                resultIndices.Add(pointIndex);
                            }
                        }
					}
				}
			}
		}

        int BinarySearchFirst(NativeArray<HashAndIndex> array, int hash) {
            int left = 0;
            int right = array.Length - 1;
            int result = -1;

            while (left <= right) {
                int mid = (left + right) / 2;
                int midHash = array[mid].Hash;

                if (midHash == hash) {
                    result = mid;
                    right = mid - 1;
                } else if (midHash < hash) {
                    left = mid + 1;
                } else {
                    right = mid - 1;
                }
            }

            return result;
        }
    }

    [BurstCompile]
    struct QueryHashJob : IJob {
        [ReadOnly] public NativeArray<HashAndIndex> hashAndIndices;
        public int queryHash;
        public NativeList<int> resultIndices;

        public void Execute() {

            int startIndex = BinarySearchFirst(hashAndIndices, queryHash);

            if (startIndex < 0) return; // No points in this grid cell

            // Loop through all the points in this grid cell that have the same hash
            for (int i = startIndex; i < hashAndIndices.Length && hashAndIndices[i].Hash == queryHash; i++) {
                int pointIndex = hashAndIndices[i].Index;
                resultIndices.Add(pointIndex);
            }
		}

        int BinarySearchFirst(NativeArray<HashAndIndex> array, int hash) {
            int left = 0;
            int right = array.Length - 1;
            int result = -1;

            while (left <= right) {
                int mid = (left + right) / 2;
                int midHash = array[mid].Hash;

                if (midHash == hash) {
                    result = mid;
                    right = mid - 1;
                } else if (midHash < hash) {
                    left = mid + 1;
                } else {
                    right = mid - 1;
                }
            }

            return result;
        }
    }

    [BurstCompile]
    struct FindPointsJob : IJob {
        [ReadOnly] public NativeArray<int> resultIndices;
        [ReadOnly] public NativeArray<Point> cloud;
        public NativeList<Point> joinedCloud;

        public void Execute() {
            foreach (int index in resultIndices) {
                joinedCloud.Add(cloud[index]);
            }
		}
    }

}

public struct Image {
    public int height;
    public int width;
    public string encoding;
    public int[] data;

    public Image(int height, int width, string encoding, byte[] byteData) {
        this.height = height;
        this.width = width;
        this.encoding = encoding;
        data = new int[height * width];
        for (int i = 0; i < data.Length; i++) {
            data[i] = (int) byteData[i];
        }
    }
}